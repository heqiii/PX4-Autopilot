#include "nmea_host_definitions.h"
#include "nmea.h"
#include "crc.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
struct Event { uint64_t when; std::string data; };
struct Feed {
 std::vector<Event> events; size_t index{}, offset{};
 std::vector<int> bauds; std::string written;
};
int callback(GPSCallbackType type,void *data,int size,void *user) {
 auto &f=*static_cast<Feed*>(user);
 if(type==GPSCallbackType::setBaudrate) {f.bauds.push_back(size);return 0;}
 if(type==GPSCallbackType::writeDeviceData) {f.written.append(static_cast<char*>(data),size);return size;}
 if(type!=GPSCallbackType::readDeviceData)return 0;
 int timeout;std::memcpy(&timeout,data,sizeof(timeout));
 const uint64_t end=host_now_us+std::min(timeout,50)*1000ULL;
 if(f.index>=f.events.size() || f.events[f.index].when>end) {host_now_us=end+1;return 0;}
 auto &e=f.events[f.index];host_now_us=std::max(host_now_us,e.when);
 const size_t n=std::min(size_t(size),e.data.size()-f.offset);
 std::memcpy(data,e.data.data()+f.offset,n);f.offset+=n;
 if(f.offset==e.data.size()) {++f.index;f.offset=0;}
 return int(n);
}
std::string nmea(const std::string &body) {
 unsigned crc=0;for(unsigned char c:body)crc^=c;
 char tail[10];std::snprintf(tail,sizeof(tail),"*%02X\r\n",crc);return "$"+body+tail;
}
std::string unicore(const std::string &body) {
 std::string copy=body;auto crc=calculateCRC32(copy.size(),reinterpret_cast<uint8_t*>(copy.data()),0);
 char tail[16];std::snprintf(tail,sizeof(tail),"*%08x\r\n",crc);return "#"+body+tail;
}
std::string heading(const char *status="SOL_COMPUTED",const char *type="NARROW_INT",const char *baseline="0.5",const char *accuracy="1.0") {
 return unicore(std::string("UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;")+status+","+type+","+baseline+",90.0,0.0,0.0,"+accuracy+",1.0,\"\",0,0,0,0,0,00,0,0");
}
std::string agrica(const char *north="1.0") {
 std::vector<std::string> fields(64,"0");fields[0]="GNSS";
 fields[22]="1.0";fields[23]=north;fields[24]="0.0";fields[25]="0.0";
 fields[26]=fields[27]=fields[28]="0.2";
 std::string body="AGRICA,97,GPS,FINE,2190,365174000,0,0,18,12;";
 for(size_t i=0;i<fields.size();i++)body+=(i?",":"")+fields[i];
 return unicore(body);
}
std::string epoch(int index,bool fix,bool utc=true,int hz=5,bool auxiliary=true) {
 char buf[30];std::snprintf(buf,sizeof(buf),"1428%02d.%02d",index/hz,index%hz*(100/hz));
 std::string time=utc?buf:"";
 if(!fix)return nmea("GNRMC,"+time+",V,,,,,,,181026,0.0,E,N,V")+nmea("GNGGA,"+time+",,,,,0,00,9999.0,,,,,,");
 auto position=nmea("GNGGA,"+time+",2250.5239,N,10813.7512,E,1,12,0.8,150.0,M,-24.1,M,,");
 if(!auxiliary)return position;
 return nmea("GNRMC,"+time+",A,2250.5239,N,10813.7512,E,1.0,90.0,181026,0.0,E,A,V")
  +position+nmea("GNGST,"+time+",1,1,1,0,0.2,0.2,0.3");
}
struct Result {int nofix{},fix{},yaw{},timeouts{};std::vector<uint64_t> timestamps;std::string startup_writes,writes;std::vector<int> bauds;};
Result run(std::vector<Event> events,bool profile=true) {
 host_now_us=1000000;Feed f;f.events=std::move(events);sensor_gps_s gps{};gps.heading=NAN;
 GPSDriverNMEA driver(callback,&f,&gps,nullptr,0.f,profile);
 unsigned baud=230400;GPSHelper::GPSConfig cfg{};cfg.output_mode=GPSHelper::OutputMode::GPS;
 assert(driver.configure(baud,cfg)==0);
 const uint64_t stop=f.events.empty()?host_now_us+2000000:f.events.back().when+200000;
 Result result;
 result.startup_writes=f.written;
 for(int guard=0;guard<1000 && host_now_us<stop;guard++) {
  int ret=driver.receive(profile?1300:500);
  if(ret<0) {result.timeouts++;continue;}
  if(ret&1) {
   result.timestamps.push_back(gps.timestamp);
   if(gps.fix_type==1) {result.nofix++;assert(!gps.vel_ned_valid && !std::isfinite(gps.heading));}
   else if(gps.fix_type>=3) {result.fix++;assert(gps.vel_ned_valid);}
   if(std::isfinite(gps.heading))result.yaw++;
   gps.heading=NAN; // actual GPS::publish clears each one-shot heading.
  }
 }
 result.writes=f.written;result.bauds=f.bauds;return result;
}
int main() {
 int cases=0;
 auto events=[](int count,unsigned interval,bool fix,bool utc){std::vector<Event> e;for(int i=0;i<count;i++)e.push_back({1000000+uint64_t(i)*interval,epoch(i,fix,utc)});return e;};
 auto r=run(events(30,200000,false,false));assert(r.nofix>=25 && r.fix==0 && r.yaw==0);cases++;
 const std::string com2_fallback = "GPGGA COM2 0.2\r\nAGRICA COM2 0.2\r\nUNIHEADINGA COM2 0.2\r\n";
 const std::string com2_commands =
  "GPGGA COM2 0.05\r\nAGRICA COM2 0.05\r\nUNIHEADINGA COM2 0.05\r\n"
  "GPGST COM2 0.2\r\nGPGSA COM2 0.2\r\nGPRMC COM2 0.2\r\n";
 assert(r.bauds==std::vector<int>{230400} && r.startup_writes==com2_fallback+com2_commands);cases++;
 assert(r.writes==com2_fallback+com2_commands+com2_commands);cases++;
 // Simulate 20 Hz position/velocity/heading with auxiliary reports at 5 Hz.
 std::vector<Event> fast{{1000000,epoch(0,false,false)}};
 for(int i=0;i<40;i++)fast.push_back({1050000+uint64_t(i)*50000,heading()+agrica()+epoch(i,true,true,20,i%4==0)});
 r=run(fast);assert(r.fix==40 && r.yaw==40 && r.nofix==1);
 for(size_t i=2;i<r.timestamps.size();i++)assert(r.timestamps[i]-r.timestamps[i-1]==50000);
 cases++;
 r=run(events(8,1000000,false,false));assert(r.nofix==8 && r.fix==0);cases++;
 r=run(events(30,200000,true,true));assert(r.fix>=25);cases++;
 auto mixed=events(30,200000,false,false);for(int i=10;i<30;i++)mixed[i].data=epoch(i,true,true);
 r=run(mixed);assert(r.nofix>=9 && r.fix>=18);cases++;
 std::vector<Event> onlyheading;for(int i=0;i<10;i++)onlyheading.push_back({1000000+uint64_t(i)*200000,heading("INSUFFICIENT_OBS","NONE","0","0")});
 r=run(onlyheading);assert(r.nofix>=8 && r.fix==0 && r.yaw==0);cases++;
 for(const auto &bad:std::vector<std::string>{heading("INSUFFICIENT_OBS","NONE","0","1"),heading("SOL_COMPUTED","NARROW_INT","0","1"),heading("SOL_COMPUTED","NARROW_INT","0.5","nan"),heading("SOL_COMPUTED","NARROW_FLOAT","0.5","1")}) {
  auto e=events(10,200000,true,true);for(auto &x:e)x.data=bad+x.data;
  r=run(e);assert(r.fix>0 && r.yaw==0);cases++;
 }
 auto good=events(10,200000,true,true);for(auto &e:good)e.data=heading()+agrica()+e.data;
 r=run(good);assert(r.fix>0 && r.yaw>0);cases++;
 auto fallback=events(30,200000,true,true);fallback[0].data=agrica()+fallback[0].data;
 r=run(fallback);assert(r.fix>15);cases++;
 auto invalid=events(10,200000,true,true);for(auto &e:invalid)e.data=agrica("nan")+e.data;
 r=run(invalid);assert(r.fix>0);cases++;
 r=run({{1000000,"garbage $GNGGA,0*00\r\n"},{2000000,"garbage"}});assert(r.fix==0 && r.nofix==0 && r.timeouts>0);cases++;
 r=run({});assert(r.nofix==0 && r.fix==0 && r.timeouts>0);cases++;
 // Explicit disconnect: a long silence must return timeout, not keep publishing last fix.
 auto gap=events(5,200000,true,true);gap.push_back({5000000,"noise"});r=run(gap);assert(r.timeouts>0);cases++;
 // Generic NMEA is not sent Unicore commands unless detected by AGRICA.
 r=run(events(6,200000,false,true),false);assert(r.writes.empty());cases++;
 auto generic_unicore=events(10,200000,true,true);
 generic_unicore[6].data=agrica()+generic_unicore[6].data;
 r=run(generic_unicore,false);
 const std::string com1_commands =
  "GPGGA COM1 0.2\r\nAGRICA COM1 0.2\r\nUNIHEADINGA COM1 0.2\r\n"
  "GPGST COM1 0.2\r\nGPGSA COM1 0.2\r\nGPRMC COM1 0.2\r\n";
 assert(r.writes==com1_commands);cases++;
 std::printf("PASS: %d NMEA/Unicore host scenarios, MyPX4 COM2 startup/retry, generic COM1 detection (no hardware)\n",cases);
}
