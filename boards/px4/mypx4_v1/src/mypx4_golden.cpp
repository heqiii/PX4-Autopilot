// MyPX4 v1.0.15 board-local golden parameter transaction.
// 黄金表原始位值同步；不得以该离线配置替代无桨复检与飞行验收。
#define MODULE_NAME "mypx4_golden"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>
#include <lib/parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <cstdint>
#include <cstring>
#include <stdlib.h>
#include "mypx4_gnss_policy.hpp"

namespace
{
struct GoldenEntry {
	const char *name;
	uint32_t bits;
	int type;
	bool unit_specific;
};
#include "mypx4_golden_values.hpp"
constexpr int32_t revision = 26;
constexpr uint8_t source_guid[] = {0, 6, 0, 0, 0, 0, 53, 57, 51, 49, 48, 51, 81, 7, 0, 79, 0, 45};

enum class Migration { Full, BaudAndNotch, Notch, FlightConfig, Rate, PitchD, Gnss, DualGnss, PlasticRate, DriverOnly };

// Keep the ~4 KB capture on the heap, away from the small NSH command stack.
struct CapturedValues {
 uint32_t *data;
 ~CapturedValues() { free(data); }
 explicit operator bool() const { return data != nullptr; }
 uint32_t &operator[](size_t index) { return data[index]; }
 uint32_t *get() { return data; }
};

bool dual_gnss_parameter(const char *name)
{
	return strcmp(name, "GPS_1_CONFIG") == 0
	    || strcmp(name, "GPS_1_PROTOCOL") == 0
	    || strcmp(name, "GPS_2_CONFIG") == 0
	    || strcmp(name, "GPS_2_PROTOCOL") == 0
	    || strcmp(name, "SER_GPS1_BAUD") == 0
	    || strcmp(name, "SER_GPS2_BAUD") == 0
	    || strcmp(name, "SENS_GPS_MASK") == 0
	    || strcmp(name, "SENS_GPS_PRIME") == 0
	    || strcmp(name, "EKF2_GPS_CTRL") == 0
	    || strcmp(name, "EKF2_MAG_TYPE") == 0;
}

bool rover_gnss_defaults()
{
	// Use the same GNSS defaults, with no quadcopter migration or writes to
	// the user layer. Installations with saved routing or offsets keep them.
	for (const auto &entry : golden) {
		if (entry.unit_specific || (!dual_gnss_parameter(entry.name)
		    && strcmp(entry.name, "GPS_YAW_OFFSET") != 0
		    && strcmp(entry.name, "EKF2_GPS_YAW_OFF") != 0)) { continue; }
		const param_t handle = param_find(entry.name);
		if (handle == PARAM_INVALID || param_type(handle) != entry.type
		    || param_set_default_value(handle, &entry.bits) != 0) {
			PX4_ERR("rover GNSS default failed: %s", entry.name);
			return false;
		}
	}
	return true;
}

bool selected(const GoldenEntry &entry, bool same_unit, Migration migration = Migration::Full)
{
	if (entry.unit_specific && !same_unit) { return false; }
	if (migration == Migration::Full) { return true; }
	if (migration == Migration::DriverOnly) { return false; }
	// Every deployed revision receives the explicitly authorized eight rate gains.
	if (strcmp(entry.name, "MC_ROLLRATE_K") == 0
	    || strcmp(entry.name, "MC_ROLLRATE_P") == 0
	    || strcmp(entry.name, "MC_ROLLRATE_I") == 0
	    || strcmp(entry.name, "MC_ROLLRATE_D") == 0
	    || strcmp(entry.name, "MC_PITCHRATE_K") == 0
	    || strcmp(entry.name, "MC_PITCHRATE_P") == 0
	    || strcmp(entry.name, "MC_PITCHRATE_I") == 0
	    || strcmp(entry.name, "MC_PITCHRATE_D") == 0) { return true; }
	if (migration == Migration::PlasticRate) { return false; }
	if (dual_gnss_parameter(entry.name)) { return true; }
	if (migration == Migration::DualGnss) { return false; }
	if (strcmp(entry.name, "GPS_YAW_OFFSET") == 0) { return true; }
	if (migration == Migration::Gnss) { return false; }
	if (strcmp(entry.name, "MC_PITCHRATE_D") == 0) { return true; }
	if (migration == Migration::PitchD) { return false; }
	if (strcmp(entry.name, "MC_ROLLRATE_K") == 0
	    || strcmp(entry.name, "MC_PITCHRATE_K") == 0) { return true; }
	if (migration == Migration::Rate) { return false; }
	return strcmp(entry.name, "SER_TEL1_BAUD") == 0
	       || strcmp(entry.name, "MAV_0_FLOW_CTRL") == 0
	       || strcmp(entry.name, "MAV_0_FORWARD") == 0
	       || strcmp(entry.name, "MPC_ALT_MODE") == 0
	       || (migration != Migration::FlightConfig && strcmp(entry.name, "IMU_GYRO_DNF_EN") == 0)
	       || (migration != Migration::FlightConfig && strcmp(entry.name, "IMU_GYRO_FFT_EN") == 0)
	       || (migration == Migration::BaudAndNotch && strcmp(entry.name, "SER_TEL3_BAUD") == 0);
}

bool read_bits(const GoldenEntry &entry, uint32_t &bits)
{
 int32_t integer = 0;
 float real = 0.f;
 const param_t handle = param_find(entry.name);
 const int result = entry.type == PARAM_TYPE_FLOAT ? param_get(handle, &real) : param_get(handle, &integer);
 memcpy(&bits, entry.type == PARAM_TYPE_FLOAT ? static_cast<const void *>(&real) : &integer, sizeof(bits));
 return result == 0;
}

bool verify(bool same_unit, const uint32_t *desired)
{
 size_t index = 0;
 for (const auto &entry : golden) {
  const size_t current = index++;
  if (!selected(entry, same_unit)) { continue; }
  uint32_t actual = 0;
  if (!read_bits(entry, actual) || actual != desired[current]) {
   PX4_ERR("golden/preserved readback mismatch: %s", entry.name);
   return false;
  }
 }
 return true;
}

bool defaults(bool same_unit, bool temporary, Migration migration = Migration::Full)
{
	for (const auto &entry : golden) {
		if (!selected(entry, same_unit, migration)) { continue; }
		// Distinct finite defaults force every target into the first BSON save,
		// including values equal to firmware defaults. Never start consumers
		// with these temporary defaults. A failure stops rcS with autosave off.
		int32_t alternate_i = entry.bits == 0 ? 1 : 0;
		float alternate_f = entry.bits == 0 ? 1.f : 0.f;
		const void *value = &entry.bits;
		if (temporary) {
			value = entry.type == PARAM_TYPE_FLOAT ? static_cast<const void *>(&alternate_f) : &alternate_i;
		}
		if (param_set_default_value(param_find(entry.name), value) != 0) { return false; }
	}
	return true;
}
} // namespace

int mypx4_golden(bool apply)
{
	// The command is an early-boot operation, never an in-flight tuning API.
	uORB::Subscription status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s status{};
	if (status_sub.copy(&status)) {
		PX4_ERR("golden command is only allowed before commander starts");
		return 1;
	}
	int32_t airframe = 0;
	int32_t previous = 0;
	const param_t marker = param_find("MYPX4_CFG_VER");
	if (param_get(param_find("SYS_AUTOSTART"), &airframe) != 0 || param_get(marker, &previous) != 0) { return 1; }
	if (airframe == mypx4::differential_rover_airframe) {
		if (apply) { return 0; } // The quadcopter transaction is not a Rover operation.
		return rover_gnss_defaults() ? 0 : 1;
	}
	if (airframe != 4001 && airframe != 4081 && airframe != 4082 && airframe != 4014) { return 0; }
	if (previous > revision) { return 0; } // Never downgrade a future profile.
	px4_guid_t guid{};
	board_get_px4_guid(guid);
	static_assert(sizeof(guid) == sizeof(source_guid), "GUID size");
	// Existing deployed profiles receive only this release's authorized delta.
	const Migration migration = previous == 17 ? Migration::BaudAndNotch
				    : previous == 18 ? Migration::Notch
				    : previous == 19 ? Migration::FlightConfig
				    : previous == 20 ? Migration::Rate
				    : previous == 21 ? Migration::PitchD
				    : previous == 22 ? Migration::Gnss
				    : previous == 23 ? Migration::DualGnss
				    : previous == 24 ? Migration::PlasticRate
                                    : previous == 25 ? Migration::DriverOnly : Migration::Full;
	const bool same_unit = memcmp(guid, source_guid, sizeof(guid)) == 0;

	// Validate the full contract before any write: missing/type errors fail closed.
	for (const auto &entry : golden) {
		if (!selected(entry, same_unit)) { continue; }
		const param_t handle = param_find(entry.name);
		if (handle == PARAM_INVALID || param_type(handle) != entry.type) {
			PX4_ERR("golden parameter missing/type mismatch: %s", entry.name);
			return 1;
		}
	}
	if (!apply || previous == revision) {
		return defaults(same_unit, false) ? 0 : 1;
	}
 // Reinstate established fallback defaults for entries omitted by earlier
 // autosaves, without replacing any explicitly stored operator value.
 if (!defaults(same_unit, false)) { return 1; }
 // Capture every applicable effective value BEFORE temporary defaults. The
 // storage writer omits values equal to runtime defaults. A limited migration
 // must serialize preserved values too, rather than verifying only its delta.
 constexpr size_t count = sizeof(golden) / sizeof(golden[0]);
 CapturedValues desired{static_cast<uint32_t *>(calloc(count, sizeof(uint32_t)))};
 if (!desired) { return 1; }
 size_t index = 0;
 for (const auto &entry : golden) {
  const size_t current = index++;
  if (!selected(entry, same_unit)) { continue; }
  if (migration != Migration::Full && selected(entry, same_unit, migration)
      && strcmp(entry.name, "EKF2_GPS_CTRL") == 0) {
   // Enable position/velocity/height/yaw, preserving any other deployed bits.
   if (!read_bits(entry, desired[current])) { return 1; }
   desired[current] |= 15u;
  } else if (selected(entry, same_unit, migration)) {
   desired[current] = entry.bits;
  } else if (!read_bits(entry, desired[current])) {
   return 1;
  }
 }
 PX4_INFO("v1.0.15 golden hard sync revision %ld -> 26, source unit=%d, migration=%d", (long)previous, same_unit, static_cast<int>(migration));
 param_control_autosave(false);
 bool ok = true;
 index = 0;
 for (const auto &entry : golden) {
  const size_t current = index++;
  if (!selected(entry, same_unit)) { continue; }
  int32_t alternate_i = desired[current] == 0 ? 1 : 0;
  float alternate_f = desired[current] == 0 ? 1.f : 0.f;
  const void *alternate = entry.type == PARAM_TYPE_FLOAT ? static_cast<const void *>(&alternate_f) : &alternate_i;
  ok = ok && param_set_default_value(param_find(entry.name), alternate) == 0;
  ok = ok && param_set_no_notification(param_find(entry.name), &desired[current]) == 0;
 }
	// Save without the completion marker, then reload the actual primary
	// parameter device. Distinct defaults expose omitted or corrupted entries.
	ok = ok && verify(same_unit, desired.get()) && param_save_default(true) == 0;
	ok = ok && param_load_default() == 0 && verify(same_unit, desired.get());
	if (ok) {
		ok = param_set_no_notification(marker, &revision) == 0 && param_save_default(true) == 0;
		ok = ok && param_load_default() == 0 && verify(same_unit, desired.get());
		int32_t loaded_revision = 0;
		ok = ok && param_get(marker, &loaded_revision) == 0 && loaded_revision == revision;
	}
	if (!ok) {
		// No background save may publish a success marker after a failed step.
		// Best effort returns the primary store to a retryable revision. Power
		// loss during a write is still subject to PX4's normal backup recovery.
		param_set_no_notification(marker, &previous);
		param_save_default(true);
		PX4_ERR("golden transaction failed; autosave disabled, reboot required");
		return 1;
	}
	if (!defaults(same_unit, false)) {
		param_set_no_notification(marker, &previous);
		param_save_default(true);
		return 1;
	}
	param_control_autosave(true);
	PX4_INFO("v1.0.15 golden hard sync saved/reloaded; revision 26 complete");
	return 0;
}
