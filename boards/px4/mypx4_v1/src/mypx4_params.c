/**
 * MyPX4 golden configuration revision
 *
 * Revision 17 (HE_PX4_v1.0.5) replaces legacy 0..16 migrations on 4001
 * and retired 4081/4082/4014 IDs. It writes the 2026-09-29 golden v3
 * snapshot once, verifies raw values and persisted reload before committing.
 * Flight counters remain untouched. Unit calibration defaults and writes
 * require the source board GUID. Later operator edits survive repeat boots.
 * Revision 18 (HE_PX4_v1.0.6) changes UART5/TELEM3 to 57600 once.
 * Revision 17 upgrades only this baud; 0..16 use v3 plus the baud override.
 * Revision 19 (HE_PX4_v1.0.8) enables FFT analysis and FFT dynamic notch.
 * Revision 18 receives only these two parameters; 17 also receives the baud.
 * Revision 19 repeat boots preserve subsequent operator edits.
 * Revision 20 (HE_PX4_v1.0.9) retains observed UART7 baud115200,
 * flow/forward disabled and sets MPC_ALT_MODE=1. Revision19 gets only
 * these four entries; all PID, calibration and other saved values survive.
 * Repeat20 preserves edits; future revisions never downgrade.
 * Revision 21 (HE_PX4_v1.0.10) applies only roll/pitch RATE_K=0.90
 * to revision20. Repeat21 preserves operator edits; >21 never downgrades.
 * No new parameter, D lowpass, PWM or calibration change is authorized.
 * Revision 22 (HE_PX4_v1.0.11) applies only pitch RATE_D=0.0033
 * to revision21. Revision20 additionally receives both K=0.90 targets.
 * Repeat22 preserves operator edits; >22 never downgrades.
 * Revision 23 (HE_PX4_v1.0.12) uses physical GPS2/UART8 as logical main
 * GPS: port202, NMEA6, baud230400, secondary disabled, yaw offset0.
 * Deployed17..22 add GPS heading bit8, retaining other EKF GPS bits.
 * Revision22 only receives the UM982 delta; repeat23 preserves edits.
 * Revision 24 (HE_PX4_v1.0.13) enables both GNSS inputs: UM982 main
 * GPS2/UART8 NMEA, original GPS1 u-blox backup; strict selection and
 * compass heading fallback. Revision23 only receives dual-GNSS settings,
 * retaining its saved installation yaw offset and all unrelated values.
 * Repeat24 preserves edits; future revisions never downgrade.
 * Revision 25 (HE_PX4_v1.0.14) restores roll/pitch RATE_K=1.00,
 * RATE_P=0.15, RATE_I=0.20 and RATE_D=0.003 for plastic propellers.
 * Revision24 receives only these eight gains; older revisions also
 * receive their applicable prior deltas. FFT and dual GNSS code stay.
 * Repeat25 preserves edits; future revisions never downgrade.
 * Revision26 (v1.0.15) changes UM982 driver only; revision25 preserves
 * all saved values. Repeat26 keeps operator edits, future versions stay.
 * This marker is not physical acceptance or flight clearance.
 *
 * @min 0
 * @max 26
 * @reboot_required true
 * @group MyPX4
 */
PARAM_DEFINE_INT32(MYPX4_CFG_VER, 0);
