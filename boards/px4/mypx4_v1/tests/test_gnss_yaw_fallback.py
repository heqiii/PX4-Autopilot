"""Exercise existing EKF yaw-control method and actual mag-heading gate.

This is a control-state test with host filter doubles, not a filter/flight test.
"""
from pathlib import Path
import argparse,re,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('source',type=Path);a=p.parse_args();root=a.source
def function(text,signature):
    start=text.index(signature);op=text.index('{',start);depth=1;end=op+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]
yaw=(root/'src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp').read_text()
control=function(yaw,'void Ekf::controlGnssYawFusion(')
mag=(root/'src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp').read_text()
start=mag.index('const bool mag_consistent_or_no_ne_aiding')
end=mag.index('&& !_control_status.flags.gnss_yaw;',start)+len('&& !_control_status.flags.gnss_yaw;')
gate=mag[start:end]
common=(root/'src/modules/ekf2/EKF/common.h').read_text()
interval=re.search(r'GNSS_YAW_MAX_INTERVAL\s*=\s*([\deE.+-]+)',common).group(1)
timeout=re.search(r'reset_timeout_max\{([\d\']+)\}',common).group(1)
fixture=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#define PX4_ISFINITE(x) std::isfinite(x)
#define ECL_INFO(...) do {} while(0)
constexpr uint64_t GNSS_YAW_MAX_INTERVAL=__INTERVAL__;
enum class GnssCtrl {YAW=8};
struct gnssSample {float yaw=.2f,yaw_offset=0;};
struct Ekf {
 struct Params{int32_t ekf2_gps_ctrl=15;uint64_t reset_timeout_max=__TIMEOUT__;}_params;
 struct Flags{bool gnss_yaw=false,gnss_yaw_fault=false,tilt_align=true,yaw_align=true,in_air=false,mag_heading_consistent=true,mag=true,mag_fault=false,mag_field_disturbed=false,ev_yaw=false;};
 struct Control{Flags flags;}_control_status;
 struct Checks{bool good=true;bool passed(){return good;}}_gnss_checks;
 struct Aid{uint64_t time_last_fuse=0;bool innovation_rejected=false;}_aid_src_gnss_yaw;
 uint64_t now=1000000,_time_last_gnss_yaw_buffer_push=1000000;
 bool _gps_intermittent=false,ne=true,reset_ok=true,fuse_ok=true;
 int resets=0,fuses=0;
 bool isNewestSampleRecent(uint64_t t,uint64_t d){return t>0&&t<=now&&now-t<=d;}
 bool isTimedOut(uint64_t t,uint64_t d){return !isNewestSampleRecent(t,d);}
 bool isNorthEastAidingActive(){return ne;}
 void updateGnssYaw(const gnssSample &){}
 void stopGnssYawFusion(){_control_status.flags.gnss_yaw=false;}
 bool resetYawToGnss(float,float){resets++;return reset_ok;}
 void resetAidSourceStatusZeroInnovation(Aid &a){a.time_last_fuse=now;a.innovation_rejected=false;}
 void fuseGnssYaw(float){fuses++;if(fuse_ok)_aid_src_gnss_yaw.time_last_fuse=now;}
 bool magAllowed(){MAG_GATE return common_conditions_passing;}
 void controlGnssYawFusion(const gnssSample &gnss_sample);
};
CONTROL
int main(){int cases=0;gnssSample sample;
 {Ekf e;assert(e.magAllowed());e.controlGnssYawFusion(sample);assert(e._control_status.flags.gnss_yaw&&!e.magAllowed());cases++;
  sample.yaw=NAN;e.now=e._time_last_gnss_yaw_buffer_push+e._params.reset_timeout_max;e.controlGnssYawFusion(sample);assert(e._control_status.flags.gnss_yaw);cases++;
  e.now++;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw&&e.magAllowed());cases++;
  e._control_status.flags.mag_field_disturbed=true;assert(!e.magAllowed());e._control_status.flags.mag_field_disturbed=false;e._control_status.flags.mag_fault=true;assert(!e.magAllowed());cases++;
 }
 sample.yaw=.2f;
 {Ekf e;e._gnss_checks.good=false;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw&&e.magAllowed());cases++;}
 {Ekf e;e._gps_intermittent=true;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw);cases++;}
 {Ekf e;e._control_status.flags.in_air=true;e._aid_src_gnss_yaw.innovation_rejected=true;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw&&e.resets==0);cases++;
  e._aid_src_gnss_yaw.innovation_rejected=false;e.controlGnssYawFusion(sample);assert(e._control_status.flags.gnss_yaw&&!e.magAllowed()&&e.resets==0);cases++;}
 {Ekf e;e._params.ekf2_gps_ctrl=7;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw&&e.magAllowed());cases++;}
 {Ekf e;e._control_status.flags.gnss_yaw_fault=true;e.controlGnssYawFusion(sample);assert(!e._control_status.flags.gnss_yaw);cases++;}
 printf("PASS: %d actual EKF yaw-control/mag-gate scenarios; existing loss timeout=%llu us\n",cases,(unsigned long long)Ekf{}. _params.reset_timeout_max);
}
'''.replace('__INTERVAL__',interval).replace('__TIMEOUT__',timeout).replace('MAG_GATE',gate).replace('CONTROL',control)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';cpp.write_text(fixture);exe=Path(d)/'test'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
