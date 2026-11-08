/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
#include <lib/parameters/param.h>
#endif
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_gps_position_pub.advertise();
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
		int32_t af = 0, main_port = 0, backup_port = 0, protocol = 0;
		mypx4::GnssCriteria criteria;
		const bool configured = param_get(param_find("SYS_AUTOSTART"), &af) == 0
			&& param_get(param_find("GPS_1_CONFIG"), &main_port) == 0
			&& param_get(param_find("GPS_2_CONFIG"), &backup_port) == 0
			&& param_get(param_find("GPS_1_PROTOCOL"), &protocol) == 0
			&& param_get(param_find("EKF2_GPS_CHECK"), &criteria.check_mask) == 0
			&& param_get(param_find("EKF2_REQ_NSATS"), &criteria.min_satellites) == 0
			&& param_get(param_find("EKF2_REQ_FIX"), &criteria.min_fix) == 0
			&& param_get(param_find("EKF2_REQ_PDOP"), &criteria.max_pdop) == 0
			&& param_get(param_find("EKF2_REQ_EPH"), &criteria.max_eph) == 0
			&& param_get(param_find("EKF2_REQ_EPV"), &criteria.max_epv) == 0
			&& param_get(param_find("EKF2_REQ_SACC"), &criteria.max_sacc) == 0;
		const bool enable = configured && mypx4::dual_gps_routes(af, main_port, backup_port, protocol)
			&& _param_sens_gps_mask.get() == 0 && _param_sens_gps_prime.get() == 0;
		if (enable != _mypx4_dual_gps) { _mypx4_gps_selector = {}; }
		_mypx4_dual_gps = enable;
		_mypx4_gps_selector.setCriteria(criteria);
		if (_mypx4_dual_gps) {
			for (auto &sub : _sensor_gps_sub) { sub.registerCallback(); }
		}
#endif

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);
			_gps_blending.setGpsData(gps_data, i);
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
			if (_mypx4_dual_gps) { _mypx4_gps_selector.setGpsData(gps_data, i); }
#endif

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
	if (_mypx4_dual_gps) {
		const int previous = _mypx4_gps_selector.selected();
		sensor_gps_s output {};
		const bool publish = _mypx4_gps_selector.update(hrt_absolute_time(), output);
		if (previous != _mypx4_gps_selector.selected()) {
			PX4_INFO("MyPX4 dual GNSS selected %d -> %d (0=UM982,1=GPS1,-1=none)",
				 previous, _mypx4_gps_selector.selected());
		}
		if (publish) { _vehicle_gps_position_pub.publish(output); }
	} else
#endif
	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			sensor_gps_s gps_output{_gps_blending.getOutputGpsData()};

			// clear device_id if blending
			if (_gps_blending.getSelectedGps() == GpsBlending::GPS_MAX_RECEIVERS_BLEND) {
				gps_output.device_id = 0;
			}

			_vehicle_gps_position_pub.publish(gps_output);
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::PrintStatus()
{
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
	if (_mypx4_dual_gps) {
		PX4_INFO_RAW("[MyPX4 dual GNSS] selected=%d reasons=%lu/%lu timeout=2s quality=1s recovery=5s\n",
			_mypx4_gps_selector.selected(), (unsigned long)_mypx4_gps_selector.reason(0),
			(unsigned long)_mypx4_gps_selector.reason(1));
		return;
	}
#endif
	PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_blending.getSelectedGps());
}

}; // namespace sensors
