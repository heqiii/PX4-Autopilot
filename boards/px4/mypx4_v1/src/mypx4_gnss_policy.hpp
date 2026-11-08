#pragma once

#include <cmath>
#include <cstdint>

namespace mypx4
{
constexpr int32_t differential_rover_airframe = 50000;

inline bool gnss_profile(int32_t airframe)
{
	return airframe == 4001 || airframe == 4081 || airframe == 4082 || airframe == 4014
	       || airframe == differential_rover_airframe;
}

inline bool dual_gps_routes(int32_t airframe, int32_t main_port, int32_t backup_port, int32_t main_protocol)
{
	return gnss_profile(airframe) && main_port == 202 && backup_port == 201 && main_protocol == 6;
}

// Reserve instance numbers without publishing a fictitious GPS measurement.
template<typename PositionPublisher, typename SatellitePublisher>
bool reserve_primary_gps_topics(PositionPublisher &position, SatellitePublisher &satellite, bool needs_satellite)
{
	return position.advertise() && position.get_instance() == 0
	       && (!needs_satellite || (satellite.advertise() && satellite.get_instance() == 0));
}

struct GnssCriteria {
	int32_t check_mask{2047};
	int32_t min_satellites{6};
	int32_t min_fix{3};
	float max_pdop{2.5f};
	float max_eph{3.f};
	float max_epv{5.f};
	float max_sacc{0.5f};
};

// Single-source selection, independent of heading availability. The EKF keeps
// its full stationary drift and innovation checks after this receiver gate.
template<typename Sample>
class DualGpsSelector
{
public:
	static constexpr uint64_t timeout_us = 2000000;
	static constexpr uint64_t quality_fail_us = 1000000;
	static constexpr uint64_t recovery_us = 5000000;
	enum Reason : uint32_t {
		Stale = 1, Fix = 2, Position = 4, Velocity = 8,
		Satellites = 16, Pdop = 32, Eph = 64, Epv = 128, Sacc = 256, Spoofed = 512
	};
	void setCriteria(const GnssCriteria &criteria) { _criteria = criteria; }
	void setGpsData(const Sample &sample, unsigned instance)
	{
		if (instance < 2 && sample.timestamp != _samples[instance].timestamp) {
			const uint64_t previous = _samples[instance].timestamp;
			if (previous != 0 && (sample.timestamp < previous || sample.timestamp - previous >= timeout_us)) {
				_good_since[instance] = _bad_since[instance] = 0;
			}
			_samples[instance] = sample;
			_updated[instance] = true;
		}
	}
	int selected() const { return _selected; }
	uint32_t reason(unsigned instance) const { return instance < 2 ? _reason[instance] : static_cast<uint32_t>(Stale); }
	bool update(uint64_t now, Sample &output)
	{
		bool healthy[2] {};
		for (unsigned i = 0; i < 2; ++i) {
			_reason[i] = assess(_samples[i], now);
			healthy[i] = _reason[i] == 0;
			if (healthy[i]) {
				if (_good_since[i] == 0) { _good_since[i] = now; }
				_bad_since[i] = 0;
			} else {
				_good_since[i] = 0;
				if (_bad_since[i] == 0) { _bad_since[i] = now; }
			}
		}
		const int previous = _selected;
		if (_selected == 0) {
			if (!healthy[0]) {
				const bool soft_failure = (_reason[0] & (Stale | Fix | Position | Velocity)) == 0;
				if (!soft_failure || now - _bad_since[0] >= quality_fail_us) {
					_selected = healthy[1] ? 1 : -1;
				}
			}
		} else if (_selected == 1) {
			if (!healthy[1]) {
				_selected = healthy[0] ? 0 : -1;
			} else if (healthy[0] && now - _good_since[0] >= recovery_us) {
				_selected = 0;
			}
		} else {
			_selected = healthy[0] ? 0 : healthy[1] ? 1 : -1;
		}
		const bool publish = _selected >= 0 && healthy[_selected]
				     && (_updated[_selected] || _selected != previous);
		if (publish) {
			output = _samples[_selected];
			if (_selected == 1) {
				// The original single-antenna GPS is a PVT backup, not a yaw source.
				output.heading = NAN;
				output.heading_offset = NAN;
				output.heading_accuracy = NAN;
			}
		}
		_updated[0] = _updated[1] = false;
		return publish;
	}

private:
	static bool finite(float value) { return std::isfinite(value); }
	static bool accuracy(float value, float limit)
	{
		return finite(value) && value >= 0.f && finite(limit) && limit > 0.f && value <= limit;
	}
	uint32_t assess(const Sample &s, uint64_t now) const
	{
		uint32_t failure = 0;
		if (s.timestamp == 0 || s.timestamp > now || now - s.timestamp >= timeout_us) { failure |= Stale; }
		const int min_fix = (_criteria.check_mask & 1024) ? _criteria.min_fix : 3;
		if (s.fix_type < 3 || s.fix_type < min_fix || s.fix_type > 6) { failure |= Fix; }
		if (!std::isfinite(s.latitude_deg) || !std::isfinite(s.longitude_deg)
		    || !std::isfinite(s.altitude_msl_m) || !std::isfinite(s.altitude_ellipsoid_m)
		    || std::fabs(s.latitude_deg) > 90.0 || std::fabs(s.longitude_deg) > 180.0) { failure |= Position; }
		if (!s.vel_ned_valid || !finite(s.vel_n_m_s) || !finite(s.vel_e_m_s) || !finite(s.vel_d_m_s)) { failure |= Velocity; }
		if ((_criteria.check_mask & 1) && s.satellites_used < _criteria.min_satellites) { failure |= Satellites; }
		if (_criteria.check_mask & 2) {
			const float pdop = std::sqrt(s.hdop * s.hdop + s.vdop * s.vdop);
			if (!finite(s.hdop) || !finite(s.vdop) || s.hdop < 0.f || s.vdop < 0.f
			    || !accuracy(pdop, _criteria.max_pdop)) { failure |= Pdop; }
		}
		if ((_criteria.check_mask & 4) && !accuracy(s.eph, _criteria.max_eph)) { failure |= Eph; }
		if ((_criteria.check_mask & 8) && !accuracy(s.epv, _criteria.max_epv)) { failure |= Epv; }
		if ((_criteria.check_mask & 16) && !accuracy(s.s_variance_m_s, _criteria.max_sacc)) { failure |= Sacc; }
		if ((_criteria.check_mask & 512) && s.spoofing_state == 3) { failure |= Spoofed; }
		return failure;
	}
	GnssCriteria _criteria {};
	Sample _samples[2] {};
	bool _updated[2] {};
	uint64_t _good_since[2] {};
	uint64_t _bad_since[2] {};
	uint32_t _reason[2] {Stale, Stale};
	int _selected{-1};
};
} // namespace mypx4
