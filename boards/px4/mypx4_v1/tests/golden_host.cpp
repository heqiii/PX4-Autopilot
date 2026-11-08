#include <cassert>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#define PX4_ERR(...) do {} while (0)
#define PX4_INFO(...) do {} while (0)
#define ORB_ID(x) 0
using param_t = int;
using px4_guid_t = uint8_t[18];
constexpr int PARAM_INVALID=-1, PARAM_TYPE_INT32=6, PARAM_TYPE_FLOAT=9;
bool commander=false, foreign=false, autosave=true;
int fail_save=0, fail_load=0, save_count=0, load_count=0, fail_set=-1, fail_default=-1;
bool corrupt_load=false;
const char *corrupt_name="BAT1_V_DIV";
struct vehicle_status_s {};
namespace uORB { struct Subscription { explicit Subscription(int) {} bool copy(vehicle_status_s *) { return commander; } }; }
struct Parameter {std::string name; uint32_t fw, def; int type;};
std::vector<Parameter> ps;
std::map<int,uint32_t> user, disk;
std::map<std::string,int> ids;
void add(std::string n,uint32_t b,int t) { ids[n]=ps.size(); ps.push_back({n,b,b,t}); }
param_t param_find(const char *n) { auto it=ids.find(n); return it==ids.end() ? -1 : it->second; }
int param_type(int p) { return ps.at(p).type; }
uint32_t value(int p) { return user.count(p) ? user[p] : ps.at(p).def; }
uint32_t bits(const char *n) { return value(param_find(n)); }
int param_get(int p,void *v) { if(p<0)return -1; auto b=value(p); memcpy(v,&b,4); return 0; }
float asfloat(uint32_t b) { float f; memcpy(&f,&b,4); return f; }
bool eq(int p,uint32_t a,uint32_t b) { return ps[p].type==6 ? a==b : fabsf(asfloat(a)-asfloat(b))<=FLT_EPSILON; }
int param_set_default_value(int p,const void *v) { if(p==fail_default)return -1; uint32_t b;memcpy(&b,v,4);ps[p].def=eq(p,b,ps[p].fw)?ps[p].fw:b;return 0; }
int param_set_no_notification(int p,const void *v) { if(p==fail_set)return -1; uint32_t b;memcpy(&b,v,4);user[p]=b;return 0; }
void put(const char *n,uint32_t b) { user[param_find(n)]=b; }
void param_control_autosave(bool enable) { autosave=enable; }
int param_save_default(bool) { if(++save_count==fail_save)return -1;disk.clear(); for(auto [p,b]:user)if(!eq(p,b,ps[p].def))disk[p]=b;return 0; }
int param_load_default() { if(++load_count==fail_load)return -1;user=disk; if(corrupt_load)user.erase(param_find(corrupt_name));return 0; }
void board_get_px4_guid(uint8_t *g) { const uint8_t id[]={0,6,0,0,0,0,53,57,51,49,48,51,81,7,0,79,0,45};memcpy(g,id,18);if(foreign)g[17]^=1; }
#include "mypx4_golden.cpp"
void boot_reset() { for(auto &p:ps)p.def=p.fw; ps[param_find("SYS_AUTOSTART")].def=4001; user=disk;save_count=load_count=0;autosave=true; }
void fresh(int rev=0,int af=4001) {disk.clear();boot_reset();commander=foreign=corrupt_load=false;corrupt_name="BAT1_V_DIV";fail_save=fail_load=0;fail_set=fail_default=-1;put("MYPX4_CFG_VER",rev);put("SYS_AUTOSTART",af);}
void expected() { /*EXPECTED*/ }
bool plastic_gain(const std::string &n) {
 return n=="MC_ROLLRATE_K" || n=="MC_ROLLRATE_P" || n=="MC_ROLLRATE_I" || n=="MC_ROLLRATE_D" || n=="MC_PITCHRATE_K" || n=="MC_PITCHRATE_P" || n=="MC_PITCHRATE_I" || n=="MC_PITCHRATE_D";
}
bool gps_delta(const std::string &n) {
 return n=="GPS_1_CONFIG" || n=="GPS_1_PROTOCOL" || n=="GPS_2_CONFIG" || n=="GPS_2_PROTOCOL" || n=="SER_GPS1_BAUD" || n=="SER_GPS2_BAUD" || n=="SENS_GPS_MASK" || n=="SENS_GPS_PRIME" || n=="EKF2_GPS_CTRL" || n=="EKF2_MAG_TYPE" || n=="GPS_YAW_OFFSET";
}
int main() {
 /*INITIAL*/
 int cases=0;
 for(int rev=0;rev<17;rev++) {
  fresh(rev);put("COM_FLIGHT_UUID",987);put("LND_FLIGHT_T_LO",0xf1234567u);
  for(const auto &e:golden) put(e.name,e.bits==0 ? 1 : 0); // corrupt the complete target set
  put("SYS_AUTOSTART",4001);
  assert(mypx4_golden(true)==0); expected();assert(bits("MYPX4_CFG_VER")==26);
  assert(bits("COM_FLIGHT_UUID")==987 && bits("LND_FLIGHT_T_LO")==0xf1234567u);
  assert(disk.size()>=960); // first commit physically exports every target plus marker
  for(int reboot=0;reboot<3;reboot++) {boot_reset();assert(mypx4_golden(true)==0);expected();assert(save_count==0);assert(param_save_default(true)==0);}
  cases++;
 }
 // Equal current values still go through the direct writer and checked save.
 fresh(16);for(const auto &e:golden)put(e.name,e.bits);assert(mypx4_golden(true)==0);expected();cases++;
 for(int af:{4081,4082,4014}) {fresh(16,af);assert(mypx4_golden(true)==0);expected();cases++;}
 // Fresh storage without a saved airframe obtains the board-selected 4001.
 fresh();user.erase(param_find("SYS_AUTOSTART"));int32_t af=4001;param_set_default_value(param_find("SYS_AUTOSTART"),&af);
 assert(mypx4_golden(true)==0);expected();cases++;
 // Edit all applicable target entries after success; all survive repeated boot/save.
 fresh(16);assert(mypx4_golden(true)==0);
 std::map<int,uint32_t> edits;
 for(const auto &e:golden) {if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART")continue;auto b=e.type==9?0x3f900000u:123u;put(e.name,b);edits[param_find(e.name)]=b;}
 assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0);
 // Simulate generic-airframe default replacement, then restore board defaults.
 float airframe_override=30.f;param_set_default_value(param_find("RTL_RETURN_ALT"),&airframe_override);
 assert(mypx4_golden(false)==0);
 for(auto [p,b]:edits) { assert(value(p)==b); } assert(save_count==0);cases++;
 fresh(16,5001);auto prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(27);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(16);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0);assert(bits("CAL_ACC0_XOFF")==0x3f900000u);assert(bits("MPC_MAN_TILT_MAX")==0x41000000u);cases++;
 for(int phase:{1,2}) {fresh(16);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==16 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0);expected();cases++;}
 for(int phase:{1,2}) {fresh(16);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==16 && !autosave);cases++;}
 fresh(16);corrupt_load=true;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==16);cases++;
 fresh(16);fail_set=param_find("MPC_MAN_TILT_MAX");assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==16);cases++;
 fresh(16);fail_default=param_find("MPC_MAN_TILT_MAX");assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==16);cases++;
 fresh(16);auto missing_id=param_find("MPC_MAN_TILT_MAX");ids.erase("MPC_MAN_TILT_MAX");prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids["MPC_MAN_TILT_MAX"]=missing_id;cases++;
 fresh(16);auto i=param_find("MPC_MAN_TILT_MAX");ps[i].type=6;prior=user;assert(mypx4_golden(true)!=0 && user==prior);ps[i].type=9;cases++;
 fresh(16);commander=true;prior=user;assert(mypx4_golden(true)!=0 && user==prior);cases++;

 // Existing revision17: preserve every other saved value including calibration
 // and edits equal to firmware defaults, while forcing and persisting one baud.
 for(int af:{4001,4081,4082,4014}) {
  fresh(17,af); std::map<int,uint32_t> keep;
  for(const auto &e:golden) {if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SER_TEL3_BAUD" || std::string(e.name)=="SYS_AUTOSTART" || std::string(e.name)=="IMU_GYRO_DNF_EN" || std::string(e.name)=="IMU_GYRO_FFT_EN" || std::string(e.name)=="SER_TEL1_BAUD" || std::string(e.name)=="MAV_0_FLOW_CTRL" || std::string(e.name)=="MAV_0_FORWARD" || std::string(e.name)=="MPC_ALT_MODE" || std::string(e.name)=="MC_ROLLRATE_K" || std::string(e.name)=="MC_PITCHRATE_K" || std::string(e.name)=="MC_PITCHRATE_D")continue; auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;}
  put("SER_TEL3_BAUD",115200);put("COM_FLIGHT_UUID",987);
  assert(mypx4_golden(true)==0 && bits("SER_TEL3_BAUD")==115200 && bits("MYPX4_CFG_VER")==26 && bits("IMU_GYRO_DNF_EN")==2 && bits("IMU_GYRO_FFT_EN")==1);
  for(auto [p,b]:keep)assert(value(p)==b);
  if(load_count==2) {for(auto [p,b]:keep)assert(disk.count(p) && disk[p]==b);}
  assert(bits("SYS_AUTOSTART")==static_cast<uint32_t>(af) && bits("COM_FLIGHT_UUID")==987);
  assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0);
  for(auto [p,b]:keep)assert(value(p)==b);
  if(load_count==2) {for(auto [p,b]:keep)assert(disk.count(p) && disk[p]==b);}
  assert(bits("SER_TEL3_BAUD")==115200);cases++;
 }
 for(int phase:{1,2}) {
  fresh(17);put("SER_TEL3_BAUD",115200);fail_save=phase;
  assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==17 && !autosave);
  fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26 && bits("SER_TEL3_BAUD")==115200);cases++;
  fresh(17);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==17 && !autosave);cases++;
 }
 fresh(17);fail_set=param_find("SER_TEL3_BAUD");assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==17);cases++;
 fresh(17);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u && bits("SER_TEL3_BAUD")==115200);cases++;
 fresh(17,5001);put("SER_TEL3_BAUD",115200);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(26);put("SER_TEL3_BAUD",38400);assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && bits("SER_TEL3_BAUD")==38400 && save_count==0);cases++;
 fresh(17);put("SER_TEL3_BAUD",115200);assert(mypx4_golden(false)==0 && bits("MYPX4_CFG_VER")==17 && bits("SER_TEL3_BAUD")==115200 && save_count==0);cases++;

 // Revision18 changes only FFT and notch; all unrelated saved/default-equal
 // values, calibration, baud and flight counters remain user-owned.
 for(int af:{4001,4081,4082,4014}) {
  fresh(18,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART" || std::string(e.name)=="IMU_GYRO_DNF_EN" || std::string(e.name)=="IMU_GYRO_FFT_EN" || std::string(e.name)=="SER_TEL1_BAUD" || std::string(e.name)=="MAV_0_FLOW_CTRL" || std::string(e.name)=="MAV_0_FORWARD" || std::string(e.name)=="MPC_ALT_MODE" || std::string(e.name)=="MC_ROLLRATE_K" || std::string(e.name)=="MC_PITCHRATE_K" || std::string(e.name)=="MC_PITCHRATE_D")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  put("SER_TEL3_BAUD",38400);keep[param_find("SER_TEL3_BAUD")]=38400;
  put("IMU_GYRO_DNF_EN",0);put("IMU_GYRO_FFT_EN",0);put("COM_FLIGHT_UUID",987);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26 && bits("IMU_GYRO_DNF_EN")==2 && bits("IMU_GYRO_FFT_EN")==1);
  for(auto [p,b]:keep)assert(value(p)==b);
  if(load_count==2) {for(auto [p,b]:keep)assert(disk.count(p) && disk[p]==b);}
  assert(bits("SYS_AUTOSTART")==static_cast<uint32_t>(af) && bits("COM_FLIGHT_UUID")==987);
  for(int reboot=0;reboot<3;reboot++) {assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);
  if(load_count==2) {for(auto [p,b]:keep)assert(disk.count(p) && disk[p]==b);}assert(bits("IMU_GYRO_DNF_EN")==2 && bits("IMU_GYRO_FFT_EN")==1);}
  // Subsequent explicit disable must survive repeated boot/save.
  put("IMU_GYRO_DNF_EN",0);put("IMU_GYRO_FFT_EN",0);assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0 && bits("IMU_GYRO_DNF_EN")==0 && bits("IMU_GYRO_FFT_EN")==0);cases++;
 }
 for(int phase:{1,2}) {
  fresh(18);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==18 && !autosave);
  fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26 && bits("IMU_GYRO_DNF_EN")==2);cases++;
  fresh(18);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==18 && !autosave);cases++;
 }
 for(const char *name:{"IMU_GYRO_DNF_EN","IMU_GYRO_FFT_EN"}) {
  fresh(18);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==18 && !autosave);cases++;
  fresh(18);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==18);cases++;
  fresh(18);auto id=param_find(name);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(18);ps[id].type=9;prior=user;assert(mypx4_golden(true)!=0 && user==prior);ps[id].type=6;cases++;
 }
 fresh(18);corrupt_load=true;corrupt_name="IMU_GYRO_DNF_EN";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==18 && !autosave);cases++;
 fresh(18);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("IMU_GYRO_DNF_EN")==2 && bits("CAL_ACC0_XOFF")==0x3f900000u);cases++;
 fresh(18,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(18);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 for(int old_revision:{17,18,19,20,21}) {
  fresh(16);assert(mypx4_golden(true)==0);expected();
  put("IMU_GYRO_DNF_EN",0);put("MYPX4_CFG_VER",old_revision);
  assert(param_save_default(true)==0);boot_reset();
  // Existing defaults are restored only on the same route as early startup.
  assert(mypx4_golden(false)==0);
  std::map<int,uint32_t> expected_saved;
  for(const auto &e:golden)expected_saved[param_find(e.name)]=bits(e.name);
  if(old_revision<19)expected_saved[param_find("IMU_GYRO_DNF_EN")]=2;
  if(old_revision<19)expected_saved[param_find("IMU_GYRO_FFT_EN")]=1;
  if(old_revision<20) {
  expected_saved[param_find("SER_TEL1_BAUD")]=115200;
  expected_saved[param_find("MAV_0_FLOW_CTRL")]=0;
  expected_saved[param_find("MAV_0_FORWARD")]=0;
  expected_saved[param_find("MPC_ALT_MODE")]=1;
  }
  if(old_revision<21) {
  expected_saved[param_find("MC_ROLLRATE_K")]=0x3f800000u;
  expected_saved[param_find("MC_PITCHRATE_K")]=0x3f800000u;
  }
  expected_saved[param_find("MC_PITCHRATE_D")]=0x3b449ba6u;
  expected_saved[param_find("MC_ROLLRATE_K")]=0x3f800000u;
  expected_saved[param_find("MC_ROLLRATE_P")]=0x3e19999au;
  expected_saved[param_find("MC_ROLLRATE_I")]=0x3e4ccccdu;
  expected_saved[param_find("MC_ROLLRATE_D")]=0x3b449ba6u;
  expected_saved[param_find("MC_PITCHRATE_K")]=0x3f800000u;
  expected_saved[param_find("MC_PITCHRATE_P")]=0x3e19999au;
  expected_saved[param_find("MC_PITCHRATE_I")]=0x3e4ccccdu;
  expected_saved[param_find("MC_PITCHRATE_D")]=0x3b449ba6u;
  if(old_revision==17)expected_saved[param_find("SER_TEL3_BAUD")]=115200;
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  for(auto [p,b]:expected_saved)assert(disk.count(p) && disk[p]==b && value(p)==b);
  // Check cold primary bytes without allowing defaults to conceal omissions.
  auto stored=disk;boot_reset();for(auto [p,b]:expected_saved)assert(stored.count(p) && value(p)==b);
  assert(mypx4_golden(true)==0);cases++;
 }

 // Revision19: only four new entries, every other raw value remains stored.
 for(int af:{4001,4081,4082,4014}) {
  fresh(19,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART" || std::string(e.name)=="SER_TEL1_BAUD" || std::string(e.name)=="MAV_0_FLOW_CTRL" || std::string(e.name)=="MAV_0_FORWARD" || std::string(e.name)=="MPC_ALT_MODE" || std::string(e.name)=="MC_ROLLRATE_K" || std::string(e.name)=="MC_PITCHRATE_K" || std::string(e.name)=="MC_PITCHRATE_D")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  put("COM_FLIGHT_UUID",321);put("LND_FLIGHT_T_HI",42);put("LND_FLIGHT_T_LO",987);
  put("MPC_ALT_MODE",2);put("SER_TEL1_BAUD",57600);put("MAV_0_FORWARD",1);put("MAV_0_FLOW_CTRL",2);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  assert(bits("MPC_ALT_MODE")==1 && bits("SER_TEL1_BAUD")==115200 && bits("MAV_0_FORWARD")==0 && bits("MAV_0_FLOW_CTRL")==0);
  for(auto [p,b]:keep)assert(value(p)==b && disk.count(p) && disk[p]==b);
  assert(bits("COM_FLIGHT_UUID")==321 && bits("LND_FLIGHT_T_HI")==42 && bits("LND_FLIGHT_T_LO")==987);
  for(int reboot=0;reboot<3;reboot++){boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);assert(bits("MPC_ALT_MODE")==1);assert(param_save_default(true)==0);}
  put("MPC_ALT_MODE",2);put("SER_TEL1_BAUD",38400);assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0 && bits("MPC_ALT_MODE")==2 && bits("SER_TEL1_BAUD")==38400);cases++;
 }
 for(int phase:{1,2}) {
  fresh(19);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==19 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);cases++;
  fresh(19);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==19 && !autosave);cases++;
 }
 for(const char *name:{"SER_TEL1_BAUD","MAV_0_FLOW_CTRL","MAV_0_FORWARD","MPC_ALT_MODE"}) {
  fresh(19);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==19 && !autosave);cases++;
  fresh(19);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==19);cases++;
 }
 fresh(19);corrupt_load=true;corrupt_name="MPC_ALT_MODE";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==19 && !autosave);cases++;
 fresh(19);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("MPC_ALT_MODE")==1 && bits("CAL_ACC0_XOFF")==0x3f900000u);cases++;
 fresh(19,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(19);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 // Revision20: inherited two K changes plus new pitch D; retain all other raw bytes.
 for(int af:{4001,4081,4082,4014}) {
  fresh(20,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART" || std::string(e.name)=="MC_ROLLRATE_K" || std::string(e.name)=="MC_PITCHRATE_K" || std::string(e.name)=="MC_PITCHRATE_D")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  put("MC_ROLLRATE_K",0x3f800000u);put("MC_PITCHRATE_K",0x3f900000u);
  put("COM_FLIGHT_UUID",654);put("LND_FLIGHT_T_HI",43);put("LND_FLIGHT_T_LO",876);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  assert(bits("MC_ROLLRATE_K")==0x3f800000u && bits("MC_PITCHRATE_K")==0x3f800000u);
  for(auto [p,b]:keep)assert(value(p)==b && disk.count(p) && disk[p]==b);
  assert(bits("COM_FLIGHT_UUID")==654 && bits("LND_FLIGHT_T_HI")==43 && bits("LND_FLIGHT_T_LO")==876);
  for(int reboot=0;reboot<3;reboot++){boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);assert(bits("MC_ROLLRATE_K")==0x3f800000u);assert(param_save_default(true)==0);}
  put("MC_ROLLRATE_K",0x3f800000u);put("MC_PITCHRATE_K",0x3f400000u);assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0 && bits("MC_ROLLRATE_K")==0x3f800000u && bits("MC_PITCHRATE_K")==0x3f400000u);cases++;
 }
 for(int phase:{1,2}) {
  fresh(20);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==20 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);cases++;
  fresh(20);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==20 && !autosave);cases++;
 }
 for(const char *name:{"MC_ROLLRATE_K","MC_PITCHRATE_K"}) {
  fresh(20);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==20 && !autosave);cases++;
  fresh(20);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==20);cases++;
  fresh(20);auto id=param_find(name);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(20);ps[id].type=6;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ps[id].type=9;cases++;
 }
 fresh(20);corrupt_load=true;corrupt_name="MC_ROLLRATE_K";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==20 && !autosave);cases++;
 fresh(20);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u && bits("MC_ROLLRATE_K")==0x3f800000u);cases++;
 fresh(20,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(20);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 // Revision21 -> 22: ONLY pitch D is forced; all 956 other golden raw values
 // (including custom K, roll D, disabled battery and PWM) remain user-owned.
 for(int af:{4001,4081,4082,4014}) {
  fresh(21,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART" || std::string(e.name)=="MC_PITCHRATE_D")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  put("MC_ROLLRATE_K",0x3f700000u);
  put("MC_PITCHRATE_K",0x3f600000u);
  put("BAT1_SOURCE",0xffffffffu);keep[param_find("BAT1_SOURCE")]=0xffffffffu;
  put("PWM_MAIN_MIN1",1150);keep[param_find("PWM_MAIN_MIN1")]=1150;
  put("MC_PITCHRATE_D",0x3b449ba6u);
  put("COM_FLIGHT_UUID",765);put("LND_FLIGHT_T_HI",44);put("LND_FLIGHT_T_LO",987);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26 && bits("MC_PITCHRATE_D")==0x3b449ba6u);
  for(auto [p,b]:keep)assert(value(p)==b && disk.count(p) && disk[p]==b);
  assert(bits("COM_FLIGHT_UUID")==765 && bits("LND_FLIGHT_T_HI")==44 && bits("LND_FLIGHT_T_LO")==987);
  for(int reboot=0;reboot<3;reboot++){boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);assert(bits("MC_PITCHRATE_D")==0x3b449ba6u);assert(param_save_default(true)==0);}
  // Current marker must preserve a subsequent explicit pitch-D edit as well.
  put("MC_PITCHRATE_D",0x3b800000u);assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0 && bits("MC_PITCHRATE_D")==0x3b800000u);cases++;
 }
 for(int phase:{1,2}) {
  fresh(21);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==21 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26 && bits("MC_PITCHRATE_D")==0x3b449ba6u);cases++;
  fresh(21);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==21 && !autosave);cases++;
 }
 {
  const char *name="MC_PITCHRATE_D";
  fresh(21);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==21 && !autosave);cases++;
  fresh(21);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==21);cases++;
  fresh(21);auto id=param_find(name);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(21);ps[id].type=6;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ps[id].type=9;cases++;
 }
 fresh(21);corrupt_load=true;corrupt_name="MC_PITCHRATE_D";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==21 && !autosave);cases++;
 fresh(21);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u && bits("MC_PITCHRATE_D")==0x3b449ba6u);cases++;
 fresh(21,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(21);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 // Revision22 receives inherited GNSS plus the authorized eight rate gains; other bits/calibration
 // remains unchanged, including values equal to firmware defaults.
 for(int af:{4001,4081,4082,4014}) {
  fresh(22,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || gps_delta(e.name) || std::string(e.name)=="SYS_AUTOSTART")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  put("BAT1_SOURCE",0xffffffffu);keep[param_find("BAT1_SOURCE")]=0xffffffffu;
  put("PWM_MAIN_MIN1",1150);keep[param_find("PWM_MAIN_MIN1")]=1150;
  put("MC_PITCHRATE_D",0x3b800000u);
  put("GPS_1_CONFIG",201);put("GPS_1_PROTOCOL",1);put("GPS_2_CONFIG",202);
  put("SER_GPS2_BAUD",57600);put("EKF2_GPS_CTRL",5);put("GPS_YAW_OFFSET",0x42b40000u);
  put("COM_FLIGHT_UUID",876);put("LND_FLIGHT_T_HI",8);put("LND_FLIGHT_T_LO",654);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  assert(bits("GPS_1_CONFIG")==202 && bits("GPS_1_PROTOCOL")==6 && bits("GPS_2_CONFIG")==201);
  assert(bits("SER_GPS2_BAUD")==230400 && bits("EKF2_GPS_CTRL")==15 && bits("GPS_YAW_OFFSET")==0);
  for(auto [p,b]:keep)assert(value(p)==b && disk.count(p) && disk[p]==b);
  assert(bits("COM_FLIGHT_UUID")==876 && bits("LND_FLIGHT_T_HI")==8 && bits("LND_FLIGHT_T_LO")==654);
  for(int n=0;n<3;n++){boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);assert(bits("EKF2_GPS_CTRL")==15);assert(param_save_default(true)==0);}
  // Later user edits of every UM982 setting survive current-marker boot.
  put("GPS_1_CONFIG",201);put("GPS_1_PROTOCOL",1);put("GPS_2_CONFIG",0);
  put("SER_GPS2_BAUD",115200);put("EKF2_GPS_CTRL",7);put("GPS_YAW_OFFSET",0x42b40000u);
  assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0);
  assert(bits("GPS_1_CONFIG")==201 && bits("SER_GPS2_BAUD")==115200 && bits("EKF2_GPS_CTRL")==7 && bits("GPS_YAW_OFFSET")==0x42b40000u);cases++;
 }
 // Cover all deployed revisions and fusion masks, including heading already set.
 for(int rev:{17,18,19,20,21,22})for(uint32_t mask=0;mask<16;mask++) {
  fresh(rev);put("EKF2_GPS_CTRL",mask);assert(mypx4_golden(true)==0);
  assert(bits("EKF2_GPS_CTRL")== (mask|15u));boot_reset();assert(mypx4_golden(true)==0 && save_count==0 && bits("EKF2_GPS_CTRL")== (mask|15u));cases++;
 }
 for(int phase:{1,2}) {
  fresh(22);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==22 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);cases++;
  fresh(22);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==22 && !autosave);cases++;
 }
 for(const char *name:{"GPS_1_CONFIG","GPS_1_PROTOCOL","GPS_2_CONFIG","SER_GPS2_BAUD","EKF2_GPS_CTRL","GPS_YAW_OFFSET"}) {
  fresh(22);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==22 && !autosave);cases++;
  fresh(22);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==22);cases++;
  fresh(22);auto id=param_find(name);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(22);auto typ=ps[id].type;ps[id].type=typ==9?6:9;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ps[id].type=typ;cases++;
 }
 fresh(22);corrupt_load=true;corrupt_name="SER_GPS2_BAUD";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==22 && !autosave);cases++;
 fresh(22);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u && bits("GPS_1_CONFIG")==202);cases++;
 fresh(22,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(22);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 // Deployed revision23: add dual GNSS plus eight rate gains; preserve all other settings and the
 // saved installation yaw offset and actual PID/PWM/battery/calibration.
 for(int af:{4001,4081,4082,4014}) {
  fresh(23,af);std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   if(plastic_gain(e.name) || (gps_delta(e.name) && std::string(e.name)!="GPS_YAW_OFFSET") || std::string(e.name)=="SYS_AUTOSTART")continue;
   auto b=ps[param_find(e.name)].fw;put(e.name,b);keep[param_find(e.name)]=b;
  }
  for(auto [name,b]:std::map<std::string,uint32_t>{{"BAT1_SOURCE",0xffffffffu},{"PWM_MAIN_MIN1",1150},{"MC_PITCHRATE_D",0x3b800000u},{"GPS_YAW_OFFSET",0x42b40000u}}) {put(name.c_str(),b);if(!plastic_gain(name))keep[param_find(name.c_str())]=b;}
  put("GPS_1_CONFIG",201);put("GPS_2_CONFIG",0);put("GPS_2_PROTOCOL",6);put("SER_GPS1_BAUD",57600);
  put("SENS_GPS_MASK",7);put("SENS_GPS_PRIME",1);put("EKF2_GPS_CTRL",0);put("EKF2_MAG_TYPE",5);
  put("COM_FLIGHT_UUID",678);put("LND_FLIGHT_T_HI",9);put("LND_FLIGHT_T_LO",432);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  assert(bits("GPS_1_CONFIG")==202 && bits("GPS_1_PROTOCOL")==6 && bits("GPS_2_CONFIG")==201 && bits("GPS_2_PROTOCOL")==1);
  assert(bits("SER_GPS1_BAUD")==0 && bits("SER_GPS2_BAUD")==230400 && bits("SENS_GPS_MASK")==0 && bits("SENS_GPS_PRIME")==0);
  assert(bits("EKF2_GPS_CTRL")==15 && bits("EKF2_MAG_TYPE")==1 && bits("GPS_YAW_OFFSET")==0x42b40000u);
  for(auto [p,b]:keep)assert(value(p)==b && disk.count(p) && disk[p]==b);
  assert(bits("COM_FLIGHT_UUID")==678 && bits("LND_FLIGHT_T_HI")==9 && bits("LND_FLIGHT_T_LO")==432);
  for(int n=0;n<3;n++){boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);assert(bits("GPS_2_CONFIG")==201 && bits("EKF2_MAG_TYPE")==1);assert(param_save_default(true)==0);}cases++;
  for(const char *name:{"GPS_1_CONFIG","GPS_1_PROTOCOL","GPS_2_CONFIG","GPS_2_PROTOCOL","SER_GPS1_BAUD","SER_GPS2_BAUD","SENS_GPS_MASK","SENS_GPS_PRIME","EKF2_GPS_CTRL","EKF2_MAG_TYPE"})put(name,123);
  assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0);
  for(const char *name:{"GPS_1_CONFIG","GPS_1_PROTOCOL","GPS_2_CONFIG","GPS_2_PROTOCOL","SER_GPS1_BAUD","SER_GPS2_BAUD","SENS_GPS_MASK","SENS_GPS_PRIME","EKF2_GPS_CTRL","EKF2_MAG_TYPE"}) { assert(bits(name)==123); }
  cases++;
 }
 for(uint32_t mask=0;mask<16;mask++){fresh(23);put("EKF2_GPS_CTRL",mask);assert(mypx4_golden(true)==0 && bits("EKF2_GPS_CTRL")== (mask|15u));cases++;}
 for(int phase:{1,2}) {
  fresh(23);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==23 && !autosave);fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);cases++;
  fresh(23);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==23 && !autosave);cases++;
 }
 for(const char *name:{"GPS_1_CONFIG","GPS_1_PROTOCOL","GPS_2_CONFIG","GPS_2_PROTOCOL","SER_GPS1_BAUD","SER_GPS2_BAUD","SENS_GPS_MASK","SENS_GPS_PRIME","EKF2_GPS_CTRL","EKF2_MAG_TYPE"}) {
  fresh(23);fail_set=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==23 && !autosave);cases++;
  fresh(23);fail_default=param_find(name);assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==23);cases++;
  fresh(23);auto id=param_find(name);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(23);auto typ=ps[id].type;ps[id].type=9;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ps[id].type=typ;cases++;
 }
 fresh(23);corrupt_load=true;corrupt_name="GPS_2_CONFIG";assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==23 && !autosave);cases++;
 fresh(23);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u && bits("GPS_2_CONFIG")==201);cases++;
 fresh(23,5001);prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(23);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;

 // Independently specify the eight user-authorized gains (not selected()).
 const std::map<std::string,uint32_t> plastic_targets={
  {"MC_ROLLRATE_K",0x3f800000u},{"MC_PITCHRATE_K",0x3f800000u},
  {"MC_ROLLRATE_P",0x3e19999au},{"MC_PITCHRATE_P",0x3e19999au},
  {"MC_ROLLRATE_I",0x3e4ccccdu},{"MC_PITCHRATE_I",0x3e4ccccdu},
  {"MC_ROLLRATE_D",0x3b449ba6u},{"MC_PITCHRATE_D",0x3b449ba6u}};
 // All deployed versions / retired IDs, with all gains deliberately wrong.
 for(int rev=17;rev<=24;rev++)for(int af:{4001,4081,4082,4014}) {
  fresh(rev,af);
  for(auto [name,b]:plastic_targets)put(name.c_str(),0x3f900000u);
  put("COM_FLIGHT_UUID",4321);put("LND_FLIGHT_T_HI",7);put("LND_FLIGHT_T_LO",1234);
  put("GPS_YAW_OFFSET",0x42b40000u);put("EKF2_GPS_CTRL",0);
  std::map<int,uint32_t> keep;
  if(rev==24)for(const auto &e:golden) {
   if(plastic_gain(e.name) || std::string(e.name)=="SYS_AUTOSTART")continue;
   const uint32_t b=e.type==9?0x3f900000u:123u;put(e.name,b);keep[param_find(e.name)]=b;
  }
  if(rev==24) {put("EKF2_GPS_CTRL",0);keep[param_find("EKF2_GPS_CTRL")]=0;}
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  for(auto [name,b]:plastic_targets)assert(bits(name.c_str())==b && disk.at(param_find(name.c_str()))==b);
  for(auto [p,b]:keep)assert(value(p)==b && disk.at(p)==b);
  if(rev<24)assert(bits("EKF2_GPS_CTRL")==15);
  if(rev==23)assert(bits("GPS_YAW_OFFSET")==0x42b40000u);
  assert(bits("SYS_AUTOSTART")==static_cast<uint32_t>(af));
  assert(bits("COM_FLIGHT_UUID")==4321 && bits("LND_FLIGHT_T_HI")==7 && bits("LND_FLIGHT_T_LO")==1234);
  for(int n=0;n<3;n++) {
   boot_reset();assert(mypx4_golden(true)==0 && save_count==0);
   for(auto [name,b]:plastic_targets)assert(bits(name.c_str())==b);
   for(auto [p,b]:keep)assert(value(p)==b);
   assert(param_save_default(true)==0);
  }
  for(auto [name,b]:plastic_targets)put(name.c_str(),0x3f900000u);
  assert(param_save_default(true)==0);boot_reset();assert(mypx4_golden(true)==0 && save_count==0);
  for(auto [name,b]:plastic_targets)assert(bits(name.c_str())==0x3f900000u);
  cases++;
 }
 // Missing source-board calibration is not imported on foreign hardware.
 fresh(24);foreign=true;put("CAL_ACC0_XOFF",0x3f900000u);
 assert(mypx4_golden(true)==0 && bits("CAL_ACC0_XOFF")==0x3f900000u);
 for(auto [name,b]:plastic_targets) { assert(bits(name.c_str())==b); } cases++;
 for(int rev:{24,25,26,27}) {
  fresh(rev,5001);for(auto [name,b]:plastic_targets)put(name.c_str(),0x3f900000u);
  prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 }
 fresh(27);for(auto [name,b]:plastic_targets)put(name.c_str(),0x3f900000u);
 prior=user;assert(mypx4_golden(true)==0 && user==prior && save_count==0);cases++;
 fresh(24);commander=true;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);cases++;
 fresh(24);prior=user;assert(mypx4_golden(false)==0 && user==prior && save_count==0);cases++;
 for(int phase:{1,2}) {
  fresh(24);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==24 && !autosave);
  fail_save=0;boot_reset();assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);cases++;
  fresh(24);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==24 && !autosave);cases++;
 }
 for(auto [name,b]:plastic_targets) {
  fresh(24);auto id=param_find(name.c_str());fail_set=id;
  assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==24 && !autosave);cases++;
  fresh(24);fail_default=id;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==24);cases++;
  fresh(24);ids.erase(name);prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ids[name]=id;cases++;
  fresh(24);ps[id].type=6;prior=user;assert(mypx4_golden(true)!=0 && user==prior && save_count==0);ps[id].type=9;cases++;
  fresh(24);corrupt_load=true;corrupt_name=name.c_str();
  assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==24 && !autosave);cases++;
 }

 // Driver-only revision25: all target and counter raw values remain user-owned.
 for(int af:{4001,4081,4082,4014}) {
  fresh(25,af); std::map<int,uint32_t> keep;
  for(const auto &e:golden) {
   const uint32_t value=std::string(e.name)=="SYS_AUTOSTART"?af:(e.type==9?0x3f900000u:123u);
   put(e.name,value);keep[param_find(e.name)]=value;
  }
  put("EKF2_GPS_CTRL",0);keep[param_find("EKF2_GPS_CTRL")]=0;
  put("COM_FLIGHT_UUID",4321);put("LND_FLIGHT_T_HI",7);put("LND_FLIGHT_T_LO",1234);
  assert(mypx4_golden(true)==0 && bits("MYPX4_CFG_VER")==26);
  for(auto [p,b]:keep) assert(value(p)==b && disk.at(p)==b);
  assert(bits("COM_FLIGHT_UUID")==4321 && bits("LND_FLIGHT_T_HI")==7 && bits("LND_FLIGHT_T_LO")==1234);
  for(int boot=0;boot<3;boot++) {boot_reset();assert(mypx4_golden(true)==0 && save_count==0);for(auto [p,b]:keep)assert(value(p)==b);}
  cases++;
 }
 for(int phase:{1,2}) {
  fresh(25);fail_save=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==25 && !autosave);cases++;
  fresh(25);fail_load=phase;assert(mypx4_golden(true)!=0 && bits("MYPX4_CFG_VER")==25 && !autosave);cases++;
 }
 // A newer firmware default must preserve both explicitly saved CFG26 values.
 fresh(26);assert(mypx4_golden(false)==0);
 put("SER_TEL3_BAUD",57600);put("EKF2_GPS_CTRL",7);
 assert(param_save_default(true)==0);boot_reset();
 assert(mypx4_golden(true)==0 && bits("SER_TEL3_BAUD")==57600 && bits("EKF2_GPS_CTRL")==7 && save_count==0);cases++;
 // Rover defaults are selected by airframe, not by the quad migration marker.
 const std::map<std::string,uint32_t> rover_gnss = {
  {"GPS_1_CONFIG",202},{"GPS_1_PROTOCOL",6},{"GPS_2_CONFIG",201},{"GPS_2_PROTOCOL",1},
  {"SER_GPS1_BAUD",0},{"SER_GPS2_BAUD",230400},{"SENS_GPS_MASK",0},{"SENS_GPS_PRIME",0},
  {"EKF2_GPS_CTRL",15},{"EKF2_MAG_TYPE",1},{"GPS_YAW_OFFSET",0},{"EKF2_GPS_YAW_OFF",0}};
 for(int rev:{0,25,26,27}) {
  fresh(rev,50000);
  put("PWM_MAIN_FUNC1",101);put("CA_R_REV",0);put("CAL_ACC0_XOFF",0x3f900000u);
  put("COM_FLIGHT_UUID",987);
  assert(param_save_default(true)==0);boot_reset();
  const auto saved=user;const auto stored=disk;
  assert(mypx4_golden(true)==0 && bits("GPS_1_CONFIG")==201 && bits("EKF2_GPS_CTRL")==7);
  assert(mypx4_golden(false)==0);
  for(size_t p=0;p<ps.size();p++) {
   auto found=rover_gnss.find(ps[p].name);
   if(ps[p].name!="SYS_AUTOSTART")assert(ps[p].def==(found==rover_gnss.end()?ps[p].fw:found->second));
  }
  for(const auto &[name,target]:rover_gnss)assert(bits(name.c_str())==target);
  assert(user==saved && disk==stored && save_count==0 && load_count==0 && autosave);
  assert(bits("SYS_AUTOSTART")==50000 && bits("MYPX4_CFG_VER")==static_cast<uint32_t>(rev));
  cases++;
 }
 // A deployed Rover can keep different routing, no-yaw and nonzero offsets.
 fresh(26,50000);assert(mypx4_golden(false)==0);
 put("GPS_1_CONFIG",201);put("GPS_1_PROTOCOL",1);put("GPS_2_CONFIG",0);
 put("SER_GPS2_BAUD",115200);put("EKF2_GPS_CTRL",7);put("GPS_YAW_OFFSET",0x42b40000u);
 put("EKF2_GPS_YAW_OFF",0x41f00000u);put("COM_FLIGHT_UUID",987);
 assert(param_save_default(true)==0);
 const auto rover_saved=disk;
 for(int reboot=0;reboot<3;reboot++) {
  boot_reset();assert(mypx4_golden(false)==0);
  for(auto [param,b]:rover_saved)assert(value(param)==b);
  assert(user==rover_saved && save_count==0 && load_count==0 && param_save_default(true)==0);
  assert(disk==rover_saved);
 }
 cases++;
 fresh(26,51000);prior=user;assert(mypx4_golden(false)==0 && bits("GPS_1_CONFIG")==201 && user==prior && save_count==0);cases++;
 fresh(26,50000);fail_default=param_find("GPS_2_CONFIG");prior=user;
 assert(mypx4_golden(false)!=0 && user==prior && save_count==0);cases++;
 printf("PASS: %d native transaction scenarios, 959 raw-bit targets; fault injection and repeat save/reboot\n",cases);
}
