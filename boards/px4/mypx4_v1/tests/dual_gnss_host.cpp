#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <atomic>
#include <limits>
#include <vector>
#include "mypx4_gnss_policy.hpp"

struct sensor_gps_s {
 uint64_t timestamp=0,timestamp_sample=0;
 uint32_t device_id=0;
 double latitude_deg=45,longitude_deg=110,altitude_msl_m=100,altitude_ellipsoid_m=120;
 float vel_n_m_s=0,vel_e_m_s=0,vel_d_m_s=0;
 float eph=.5f,epv=1.f,s_variance_m_s=.1f,hdop=.6f,vdop=.8f;
 float heading=.5f,heading_offset=0,heading_accuracy=.01f,rtcm_injection_rate=0;
 uint8_t fix_type=3,satellites_used=12,spoofing_state=1,selected_rtcm_instance=0;
 bool vel_ned_valid=true;
};
sensor_gps_s sample(uint64_t t,unsigned i=0) {
 sensor_gps_s s;s.timestamp=s.timestamp_sample=t;s.device_id=100+i;s.latitude_deg=45+i;return s;
}
using Selector=mypx4::DualGpsSelector<sensor_gps_s>;
int cases=0;
void selector_tests() {
 sensor_gps_s out;
 for(int af:{4001,4081,4082,4014,50000}){
  assert(mypx4::dual_gps_routes(af,202,201,6));
  assert(!mypx4::dual_gps_routes(af,201,202,6));
  assert(!mypx4::dual_gps_routes(af,202,0,6));
  assert(!mypx4::dual_gps_routes(af,202,201,1));cases++;
 }
 for(int af:{5001,50001,51000,52000}){assert(!mypx4::dual_gps_routes(af,202,201,6));cases++;}
 {Selector x;assert(!x.update(1000000,out)&&x.selected()==-1);cases++;}
 // Both available: prefer ordinary UM982 fix3 over backup RTK fix6. No averaging.
 {Selector x;auto a=sample(1000000),b=sample(1000000,1);b.fix_type=6;x.setGpsData(a,0);x.setGpsData(b,1);
  assert(x.update(1000000,out)&&x.selected()==0&&out.latitude_deg==45&&out.timestamp==a.timestamp);cases++;
  a=sample(1200000);a.heading=NAN;x.setGpsData(a,0);assert(x.update(1200000,out)&&x.selected()==0&&std::isnan(out.heading));cases++;
  assert(!x.update(1300000,out));x.setGpsData(a,0);assert(!x.update(1300000,out));cases++;}
 // Each receiver works alone from cold boot; backup never supplies its own yaw.
 for(unsigned i:{0u,1u}){Selector x;x.setGpsData(sample(1000000,i),i);assert(x.update(1000000,out)&&x.selected()==int(i));assert(out.device_id==100+i);if(i==1)assert(std::isnan(out.heading)&&std::isnan(out.heading_offset)&&std::isnan(out.heading_accuracy));cases++;}
 // Lost primary times out at2s while backup updates; both lost suppress output.
 {Selector x;x.setGpsData(sample(1000000),0);x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out));
  x.setGpsData(sample(2999999,1),1);assert(!x.update(2999999,out)&&x.selected()==0);
  x.setGpsData(sample(3000000,1),1);assert(x.update(3000000,out)&&x.selected()==1&&out.timestamp==3000000);cases++;
  assert(!x.update(5000000,out)&&x.selected()==-1);cases++;}
 // Sustained poor accuracy: stop forwarding bad samples; switch after1s.
 {Selector x;x.setGpsData(sample(1000000),0);x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out));
  for(uint64_t t=1200000;t<=2200000;t+=200000){auto a=sample(t);a.eph=20;x.setGpsData(a,0);x.setGpsData(sample(t,1),1);bool p=x.update(t,out);if(t<2200000)assert(!p&&x.selected()==0);else assert(p&&x.selected()==1);}
  cases++;
  // Continuous recovery needs5s; one brief failure restarts this interval.
  for(uint64_t t=2400000;t<=7400000;t+=200000){x.setGpsData(sample(t),0);x.setGpsData(sample(t,1),1);assert(x.update(t,out));assert(x.selected()==(t<7400000?1:0));}cases++;}
 {Selector x;x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out));
  for(uint64_t t=1200000;t<=4000000;t+=200000){x.setGpsData(sample(t),0);x.setGpsData(sample(t,1),1);assert(x.update(t,out)&&x.selected()==1);}
  auto bad=sample(4200000);bad.epv=20;x.setGpsData(bad,0);x.setGpsData(sample(4200000,1),1);assert(x.update(4200000,out)&&x.selected()==1);
  for(uint64_t t=4400000;t<=9400000;t+=200000){x.setGpsData(sample(t),0);x.setGpsData(sample(t,1),1);assert(x.update(t,out));assert(x.selected()==(t<9400000?1:0));}cases++;}
 // Scheduling/arrival gaps cannot count as continuous recovery.
 {Selector x;x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out));x.setGpsData(sample(1200000),0);x.setGpsData(sample(1200000,1),1);assert(x.update(1200000,out));
  x.setGpsData(sample(7200000),0);x.setGpsData(sample(7200000,1),1);assert(x.update(7200000,out)&&x.selected()==1);cases++;}
 // Invalid fix/coordinates/velocity are immediate; backup must itself qualify.
 for(int failure=0;failure<7;failure++){Selector x;x.setGpsData(sample(1000000),0);assert(x.update(1000000,out));auto a=sample(1200000);
  if(failure==0)a.fix_type=0;if(failure==1)a.fix_type=2;if(failure==2)a.fix_type=8;if(failure==3)a.latitude_deg=NAN;if(failure==4)a.longitude_deg=200;if(failure==5)a.vel_ned_valid=false;if(failure==6)a.vel_d_m_s=INFINITY;
  x.setGpsData(a,0);x.setGpsData(sample(1200000,1),1);assert(x.update(1200000,out)&&x.selected()==1);cases++;}
 for(int field=0;field<8;field++){Selector x;auto a=sample(1000000);if(field==0)a.satellites_used=3;if(field==1)a.hdop=10;if(field==2)a.eph=10;if(field==3)a.epv=10;if(field==4)a.s_variance_m_s=10;if(field==5)a.spoofing_state=3;if(field==6)a.eph=NAN;if(field==7)a.timestamp=2000000;
  x.setGpsData(a,0);x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out)&&x.selected()==1&&x.reason(0)!=0);cases++;}
 // Use existing configured static thresholds; do not silently ignore enabled gates.
 {Selector x;mypx4::GnssCriteria c;c.max_eph=.2f;x.setCriteria(c);x.setGpsData(sample(1000000),0);assert(!x.update(1000000,out));cases++;}
 {Selector x;mypx4::GnssCriteria c;c.check_mask=0;x.setCriteria(c);auto a=sample(1000000);a.eph=20;a.satellites_used=3;x.setGpsData(a,0);assert(x.update(1000000,out));cases++;}
 {Selector x;mypx4::GnssCriteria c;c.min_fix=6;x.setCriteria(c);x.setGpsData(sample(1000000),0);auto b=sample(1000000,1);b.fix_type=6;x.setGpsData(b,1);assert(x.update(1000000,out)&&x.selected()==1);cases++;}
 // If backup fails, don't wait5s to use an already qualified primary.
 {Selector x;x.setGpsData(sample(1000000,1),1);assert(x.update(1000000,out));auto b=sample(1200000,1);b.fix_type=0;x.setGpsData(sample(1200000),0);x.setGpsData(b,1);assert(x.update(1200000,out)&&x.selected()==0);cases++;}
}

// Compile the actual PublicationMulti implementation with a host uORB backend.
enum class ORB_ID { gps=0, satellites=1 };
struct orb_metadata { int o_id; };
struct Handle {int topic,instance;};
Handle handles[20];int used=0,next_instance[2]={0,0},published=0;bool fail_advertise=false;
sensor_gps_s published_sample;int published_instance=-1;
constexpr int PX4_OK=0;
void *orb_advertise_multi(const orb_metadata *m,const void *data,int *instance){assert(data==nullptr);if(fail_advertise)return nullptr;*instance=next_instance[m->o_id]++;handles[used]={m->o_id,*instance};return &handles[used++];}
int orb_publish(const orb_metadata *,void *h,const void *data){assert(h);published++;published_sample=*static_cast<const sensor_gps_s *>(data);published_instance=static_cast<Handle *>(h)->instance;return PX4_OK;}
namespace uORB {
struct PublicationBase { explicit PublicationBase(ORB_ID i):id(i){};bool advertised(){return _handle!=nullptr;}const orb_metadata *get_topic(){static orb_metadata m[2]={{0},{1}};return &m[int(id)];}void *_handle=nullptr;ORB_ID id;};
struct Manager {static int orb_get_instance(void *h){return static_cast<Handle *>(h)->instance;}};
/*PUBLICATION_MULTI*/
}
enum class gps_driver_mode_t { UBX, NMEA };
struct GPS {
 gps_driver_mode_t _mode{gps_driver_mode_t::UBX};
 enum class Instance {Main=0,Secondary=1};
 Instance _instance;
 sensor_gps_s _sensor_gps=sample(1000000);
 uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID::gps},_report_sat_info_pub{ORB_ID::satellites};
 float _rtcm_injection_rate=0;uint8_t _selected_rtcm_instance=0;
 static std::atomic_bool _is_gps_main_advertised;
 explicit GPS(Instance i):_instance(i){}
 uint32_t get_device_id(){return 100+int(_instance);}
 void publish();
};
std::atomic_bool GPS::_is_gps_main_advertised{false};
/*GPS_PUBLISH*/
void publication_tests(){
 {GPS main(GPS::Instance::Main),backup(GPS::Instance::Secondary);assert(mypx4::reserve_primary_gps_topics(main._sensor_gps_pub,main._report_sat_info_pub,true));assert(published==0);GPS::_is_gps_main_advertised.store(true);
  backup.publish();assert(published==1&&published_instance==1&&published_sample.device_id==101);assert(main._sensor_gps_pub.get_instance()==0);cases++;
  main.publish();assert(published==2&&published_instance==0);cases++;}
 used=0;next_instance[0]=next_instance[1]=0;published=0;fail_advertise=true;
 {GPS main(GPS::Instance::Main);assert(!mypx4::reserve_primary_gps_topics(main._sensor_gps_pub,main._report_sat_info_pub,false));assert(published==0);cases++;}
 fail_advertise=false;next_instance[0]=1;
 {GPS main(GPS::Instance::Main);assert(!mypx4::reserve_primary_gps_topics(main._sensor_gps_pub,main._report_sat_info_pub,false));cases++;}
}

uint64_t now_us=1000000;
constexpr uint64_t operator ""_ms(unsigned long long x){return x*1000;}
uint64_t hrt_absolute_time(){return now_us;}
void perf_begin(int){}void perf_end(int){}
#define PX4_INFO(...) do {} while(0)
struct GpsBlending {
 static constexpr int GPS_MAX_RECEIVERS_BLEND=2;
 int updates=0;void setGpsData(const sensor_gps_s &,int){}void update(uint64_t){updates++;}
 bool isNewOutputDataAvailable(){return false;}sensor_gps_s getOutputGpsData(){return {};}int getSelectedGps(){return 0;}
};
struct VehicleGPSPosition {
 static constexpr int GPS_MAX_RECEIVERS=2;int _cycle_perf=0;GpsBlending _gps_blending;
 Selector _mypx4_gps_selector;bool _mypx4_dual_gps=true;
 struct Sub {bool pending=false;sensor_gps_s s;bool updated(){return pending;}void copy(sensor_gps_s *out){*out=s;pending=false;}bool registered(){return true;}void registerCallback(){}};
 Sub _sensor_gps_sub[2];
 struct Pub{int count=0;sensor_gps_s last;void publish(const sensor_gps_s &s){last=s;count++;}}_vehicle_gps_position_pub;
 void ParametersUpdate(){}void ScheduleDelayed(uint64_t){}void Run();
};
/*VEHICLE_GPS_RUN*/
void integration_tests(){
 VehicleGPSPosition v;v._sensor_gps_sub[1].s=sample(now_us,1);v._sensor_gps_sub[1].pending=true;v.Run();
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
 assert(v._vehicle_gps_position_pub.count==1&&v._vehicle_gps_position_pub.last.device_id==101&&v._gps_blending.updates==0);cases++;
 now_us+=Selector::timeout_us;v.Run();assert(v._mypx4_gps_selector.selected()==-1&&v._vehicle_gps_position_pub.count==1);cases++;
 v._mypx4_dual_gps=false;v._sensor_gps_sub[0].pending=true;v.Run();assert(v._gps_blending.updates==1);cases++;
#else
 assert(v._vehicle_gps_position_pub.count==0&&v._gps_blending.updates==1);cases++;
#endif
}
int main(){selector_tests();publication_tests();integration_tests();printf("PASS: %d native dual GNSS policy/actual publication/actual Run scenarios\n",cases);}
