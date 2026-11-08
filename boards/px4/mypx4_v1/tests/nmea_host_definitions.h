#pragma once
#include <cstdint>
#include <cstdio>
#include <ctime>
// Host-only field containers and deterministic clock; no UART/uORB/hardware.
inline uint64_t host_now_us = 100000;
using gps_abstime = uint64_t;
inline uint64_t gps_absolute_time() { return host_now_us; }
inline void gps_usleep(unsigned us) { host_now_us += us; }
#define GPS_WARN(...) ((void)0)
#define GPS_INFO(...) ((void)0)
#define GPS_ERR(...) ((void)0)
struct sensor_gps_s {
    uint32_t device_id{};
    int selected_rtcm_instance{};
    float rtcm_injection_rate{};
 uint64_t timestamp{}, time_utc_usec{};
 int32_t timestamp_time_relative{};
 double latitude_deg{}, longitude_deg{}, altitude_msl_m{}, altitude_ellipsoid_m{};
 float hdop{}, vdop{}, eph{}, epv{}, c_variance_rad{}, s_variance_m_s{};
 float vel_m_s{}, vel_n_m_s{}, vel_e_m_s{}, vel_d_m_s{}, cog_rad{}, heading{}, heading_accuracy{};
 uint8_t fix_type{}, satellites_used{};
 bool vel_ned_valid{};
};
struct sensor_gnss_relative_s {};
struct satellite_info_s {
 static constexpr unsigned SAT_INFO_MAX_SATELLITES=20;
 uint64_t timestamp{};
 uint8_t count{}, svid[20]{}, used[20]{}, snr[20]{}, elevation[20]{}, azimuth[20]{};
};
