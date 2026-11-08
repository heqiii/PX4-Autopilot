"""Exercise MyPX4 GPS publication and the real GPS_RAW_INT stream with host substitutes."""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("source", type=Path)
source = parser.parse_args().source

stream = (source / "src/modules/mavlink/streams/GPS_RAW_INT.hpp").read_text()
stream = re.sub(r"^#include .+\n", "", stream, flags=re.MULTILINE)
driver = (source / "src/drivers/gps/gps.cpp").read_text()
driver_publish = driver.split("GPS::publish()\n", 1)[1].split("\nvoid\n", 1)[0]

fixture = r'''
#include "nmea_host_definitions.h"
#include "mypx4_gnss_policy.hpp"
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>

#define PX4_ISFINITE std::isfinite
#define ORB_ID(topic) 0
using hrt_abstime = uint64_t;
namespace time_literals {
constexpr hrt_abstime operator""_s(unsigned long long seconds) { return seconds * 1000000; }
}
hrt_abstime hrt_absolute_time() { return host_now_us; }
namespace math { float degrees(float rad) { return rad * 180.f / 3.14159265f; } }
namespace matrix { float wrap_2pi(float rad) { return rad < 0 ? rad + 6.2831853f : rad; } }

constexpr int GPS_FIX_TYPE_NO_GPS = 0;
constexpr int GPS_FIX_TYPE_NO_FIX = 1;
constexpr int MAVLINK_MSG_ID_GPS_RAW_INT = 24;
constexpr int MAVLINK_MSG_ID_GPS_RAW_INT_LEN = 52;
constexpr int MAVLINK_NUM_NON_PAYLOAD_BYTES = 12;
struct mavlink_gps_raw_int_t {
    uint64_t time_usec{};
    int fix_type{}, lat{}, lon{}, alt{}, eph{}, epv{}, vel{}, cog{}, satellites_visible{};
    int alt_ellipsoid{}, h_acc{}, v_acc{}, vel_acc{}, yaw{}, hdg_acc{};
};
mavlink_gps_raw_int_t last;
void mavlink_msg_gps_raw_int_send_struct(int, const mavlink_gps_raw_int_t *msg) { last = *msg; }

const char *param_find(const char *name) { return name; }
int airframe = 4001;
int param_get(const char *name, int32_t *value)
{
    if (!std::strcmp(name, "SYS_AUTOSTART")) { *value = airframe; }
    else if (!std::strcmp(name, "GPS_1_CONFIG")) { *value = 202; }
    else if (!std::strcmp(name, "GPS_2_CONFIG")) { *value = 201; }
    else if (!std::strcmp(name, "GPS_1_PROTOCOL")) { *value = 6; }
    else { return -1; }
    return 0;
}

struct Topic {
    sensor_gps_s sample{};
    unsigned generation{};
    bool advertised{};
};
Topic topics[2];
namespace uORB {
class Subscription {
public:
    Subscription(int, int instance) : _topic(topics[instance]) {}
    bool advertised() const { return _topic.advertised; }
    bool update(sensor_gps_s *sample)
    {
        if (_generation == _topic.generation) { return false; }
        _generation = _topic.generation;
        *sample = _topic.sample;
        return true;
    }
private:
    Topic &_topic;
    unsigned _generation{};
};
}
struct Mavlink { int get_channel() const { return 0; } };
struct MavlinkStream {
    explicit MavlinkStream(Mavlink *link) : _mavlink(link) {}
    virtual ~MavlinkStream() = default;
    virtual const char *get_name() const = 0;
    virtual uint16_t get_id() = 0;
    virtual unsigned get_size() = 0;
    virtual bool send() = 0;
    Mavlink *_mavlink;
};
/*STREAM*/

enum class gps_driver_mode_t { UBX, NMEA };
struct GPS {
    enum class Instance { Main, Secondary };
    struct {
        bool load() const { return true; }
        void store(bool) {}
    } _is_gps_main_advertised;
    struct Publisher {
        unsigned instance{};
        void publish(const sensor_gps_s &gps)
        {
            auto &topic = topics[instance];
            topic.sample = gps;
            topic.advertised = true;
            ++topic.generation;
        }
    } _sensor_gps_pub;
    sensor_gps_s _sensor_gps{};
    Instance _instance{Instance::Secondary};
    gps_driver_mode_t _mode{gps_driver_mode_t::UBX};
    int _selected_rtcm_instance{};
    float _rtcm_injection_rate{};
    uint32_t get_device_id() const { return 10813445; }
    void publish() /*PUBLISH*/
};

void publish(GPS &driver, uint8_t fix, int sats = 0)
{
    driver._sensor_gps.timestamp = host_now_us;
    driver._sensor_gps.fix_type = fix;
    driver._sensor_gps.satellites_used = sats;
    driver._sensor_gps.heading = 0.5f;
    driver.publish();
}

int main(int argc, char **argv)
{
    if (argc > 1) { airframe = std::atoi(argv[1]); }
    Mavlink link;
    auto *stream = MavlinkStreamGPSRawInt::new_instance(&link);
    GPS primary, backup;
    primary._instance = GPS::Instance::Main;
    primary._mode = gps_driver_mode_t::NMEA;
    backup._sensor_gps_pub.instance = 1;

    host_now_us = 1000000;
    topics[0].advertised = true; // Reserved instance, no data.
    assert(!stream->send());
    backup._sensor_gps.eph = 4294967.5f;
    publish(backup, 0);

#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
    assert(topics[1].sample.fix_type == 1);
    assert(stream->get_size() && stream->send());
    assert(last.fix_type == 1 && last.satellites_visible == 0 && last.yaw == 0);
    assert(last.eph == UINT16_MAX && last.cog == UINT16_MAX && last.h_acc == 0);
    topics[0].advertised = false;
    assert(stream->get_size() && !stream->send());
    host_now_us += 100000;
    publish(backup, 3, 10);
    assert(stream->send() && last.fix_type == 3 && last.satellites_visible == 10 && last.yaw == 0);

    host_now_us += 100000;
    publish(primary, 6, 18);
    assert(stream->send() && last.fix_type == 6 && last.satellites_visible == 18 && last.yaw > 0);
    host_now_us += 1300000;
    publish(backup, 3, 10);
    assert(!stream->send()); // 1 Hz primary jitter cannot switch telemetry to the backup.
    host_now_us += 700001;
    assert(stream->send() && last.fix_type == 3 && last.satellites_visible == 10);
    host_now_us += 1300000;
    assert(stream->send() && last.fix_type == 0 && last.satellites_visible == UINT8_MAX);
    assert(!stream->send());

    host_now_us += 100000;
    publish(backup, 0);
    assert(stream->send() && last.fix_type == 1);
    host_now_us += 100000;
    publish(primary, 1);
    assert(stream->send() && last.fix_type == 1 && last.yaw > 0);
    delete stream;

    const int tested_airframe = airframe;
    airframe = 51000;
    topics[0] = {};
    stream = MavlinkStreamGPSRawInt::new_instance(&link);
    assert(!stream->get_size() && !stream->send());
    std::printf("PASS: MyPX4 %d backup no-fix, 3D, primary return, jitter, failover, disconnect, reconnect; 51000 unaffected\n",
                tested_airframe);
#else
    assert(topics[1].sample.fix_type == 0);
    assert(!stream->send());
    publish(primary, 3, 12);
    assert(stream->send() && last.fix_type == 3 && last.satellites_visible == 12);
    host_now_us += 1000001;
    assert(stream->send() && last.fix_type == 0);
    std::puts("PASS: non-MyPX4 primary-only stream and original timeout");
#endif
    delete stream;
}
'''

fixture = fixture.replace("/*STREAM*/", stream).replace("/*PUBLISH*/", driver_publish)
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "presence.cpp"
    exe = Path(directory) / "presence"
    cpp.write_text(fixture)
    board = source / "boards/px4/mypx4_v1"
    for defines in (["-DCONFIG_ARCH_BOARD_PX4_MYPX4_V1"], []):
        subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", *defines,
             "-I", str(board / "src"), "-I", str(board / "tests"), str(cpp), "-o", str(exe)],
            check=True,
        )
        subprocess.run([str(exe)], check=True)
        if defines:
            subprocess.run([str(exe), "50000"], check=True)
