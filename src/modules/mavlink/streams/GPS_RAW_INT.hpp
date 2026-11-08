/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef GPS_RAW_INT_HPP
#define GPS_RAW_INT_HPP

#include <uORB/topics/sensor_gps.h>

#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
#include <lib/parameters/param.h>
#include "mypx4_gnss_policy.hpp"
#endif

using namespace time_literals;

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSRawInt(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_RAW_INT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_RAW_INT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		bool advertised = _sensor_gps_sub.advertised();
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
		advertised = advertised || (_report_backup && _backup_gps_sub.advertised());
#endif
		return advertised ? (MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};
	hrt_abstime _last_send_ts {};
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
	static constexpr hrt_abstime kNoGpsSendInterval {2_s}; // Allow the primary's 1 Hz output jitter.
	uORB::Subscription _backup_gps_sub{ORB_ID(sensor_gps), 1};
	hrt_abstime _primary_sample_ts {};
	const bool _report_backup {dual_gps_configured()};

	static bool dual_gps_configured()
	{
		int32_t airframe = 0, main_port = 0, backup_port = 0, protocol = 0;
		return param_get(param_find("SYS_AUTOSTART"), &airframe) == 0
		       && param_get(param_find("GPS_1_CONFIG"), &main_port) == 0
		       && param_get(param_find("GPS_2_CONFIG"), &backup_port) == 0
		       && param_get(param_find("GPS_1_PROTOCOL"), &protocol) == 0
		       && mypx4::dual_gps_routes(airframe, main_port, backup_port, protocol);
	}

	static bool fresh(hrt_abstime timestamp)
	{
		const hrt_abstime now = hrt_absolute_time();
		return timestamp != 0 && timestamp <= now && now - timestamp <= kNoGpsSendInterval;
	}
#else
	static constexpr hrt_abstime kNoGpsSendInterval {1_s};
#endif

	bool update_gps(sensor_gps_s &gps)
	{
		const bool primary_updated = _sensor_gps_sub.update(&gps);
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)

		if (_report_backup) {
			if (primary_updated) {
				_primary_sample_ts = gps.timestamp;
			}

			if (!fresh(_primary_sample_ts)) {
				if (_backup_gps_sub.update(&gps) && fresh(gps.timestamp)) {
					// The backup supplies PVT. The estimator has its own receiver selection.
					gps.heading = NAN;
					return true;
				}

				return false;
			}
		}

#endif
		return primary_updated;
	}

	bool send() override
	{
		sensor_gps_s gps;
		mavlink_gps_raw_int_t msg{};
		hrt_abstime now{};

		if (update_gps(gps)) {
			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.satellites_visible = gps.satellites_used;
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)

			if (_report_backup && gps.fix_type == GPS_FIX_TYPE_NO_FIX) {
				// The receiver is present, but its position and velocity fields are unavailable.
				msg.eph = UINT16_MAX;
				msg.epv = UINT16_MAX;
				msg.vel = UINT16_MAX;
				msg.cog = UINT16_MAX;

			} else
#endif
			{
				msg.lat = static_cast<int32_t>(round(gps.latitude_deg * 1e7));
				msg.lon = static_cast<int32_t>(round(gps.longitude_deg * 1e7));
				msg.alt = static_cast<int32_t>(round(gps.altitude_msl_m * 1e3)); // convert [m] to [mm]
				msg.eph = gps.hdop * 100; // GPS HDOP horizontal dilution of position (unitless)
				msg.epv = gps.vdop * 100; // GPS VDOP vertical dilution of position (unitless)

				if (PX4_ISFINITE(gps.vel_m_s) && (fabsf(gps.vel_m_s) >= 0.f)) {
					msg.vel = gps.vel_m_s * 100.f; // cm/s

				} else {
					msg.vel = UINT16_MAX; // If unknown, set to: UINT16_MAX
				}

				msg.cog = math::degrees(matrix::wrap_2pi(gps.cog_rad)) * 1e2f;
				msg.alt_ellipsoid = static_cast<int32_t>(round(gps.altitude_ellipsoid_m * 1e3)); // convert [m] to [mm]
				msg.h_acc = gps.eph * 1e3f;              // position uncertainty in mm
				msg.v_acc = gps.epv * 1e3f;              // altitude uncertainty in mm
				msg.vel_acc = gps.s_variance_m_s * 1e3f; // speed uncertainty in mm
			}

			if (PX4_ISFINITE(gps.heading)) {
				if (fabsf(gps.heading) < FLT_EPSILON) {
					msg.yaw = 36000; // Use 36000 for north.

				} else {
					msg.yaw = math::degrees(matrix::wrap_2pi(gps.heading)) * 100.0f; // centidegrees
				}

				if (PX4_ISFINITE(gps.heading_accuracy)) {
					msg.hdg_acc = math::degrees(gps.heading_accuracy) * 1e5f; // Heading / track uncertainty in degE5
				}
			}

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = gps.timestamp;

			return true;

		} else if (_last_send_ts != 0 && (now = hrt_absolute_time()) > _last_send_ts + kNoGpsSendInterval) {
			msg.fix_type = GPS_FIX_TYPE_NO_GPS;
			msg.eph = UINT16_MAX;
			msg.epv = UINT16_MAX;
			msg.vel = UINT16_MAX;
			msg.cog = UINT16_MAX;
			msg.satellites_visible = UINT8_MAX;
			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = now;

			return true;
		}

		return false;
	}
};

#endif // GPS_RAW_INT_HPP
