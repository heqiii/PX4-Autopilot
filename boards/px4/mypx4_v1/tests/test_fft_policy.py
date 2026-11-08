"""Compile the actual FFT update method against recorded API doubles.

Checks board/profile scope, both IMU rates, center tracking, width-only updates,
enable/device/rate/freshness guards. This is not a closed-loop flight test.
"""
from pathlib import Path
import argparse
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
root = parser.parse_args().source
cpp = (root / 'src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp').read_text()
start = cpp.index('void VehicleAngularVelocity::UpdateDynamicNotchFFT(')
end = cpp.index('\nfloat VehicleAngularVelocity::FilterAngularVelocity', start)
method = cpp[start:end]
fixture = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "mypx4_fft_policy.hpp"
namespace math { template<typename T> T constrain(T v,T a,T b){return v<a?a:(v>b?b:v);} }
using hrt_abstime=uint64_t;
void perf_count(int) {}
struct sensor_gyro_fft_s {
 uint64_t timestamp=100;
 uint32_t device_id=123;
 float sensor_sample_rate_hz=2000, resolution_hz=3.9f;
 float peak_frequencies_x[3]={40,85,130},peak_frequencies_y[3]={40,85,130},peak_frequencies_z[3]={40,85,130};
};
struct Subscription {
 sensor_gyro_fft_s data;
 bool valid=true;
 bool updated(){return true;}
 bool copy(sensor_gyro_fft_s *out){*out=data;return valid;}
};
struct Param {int value;int get(){return value;}};
struct Notch {
 float freq=0,bw=0;int updates=0;
 float getNotchFreq(){return freq;}
 float getBandwidth(){return bw;}
 bool initialized(){return updates>0;}
 void disable(){freq=0;}
 void setParameters(float,float f,float b){freq=f;bw=b;updates++;}
};
class VehicleAngularVelocity {
public:
 struct DynamicNotch {static constexpr int FFT=2;};
 static constexpr uint64_t DYNAMIC_NOTCH_FITLER_TIMEOUT=3000000;
 static constexpr int MAX_NUM_FFT_PEAKS=3;
 Param _param_imu_gyro_dnf_en{2},_param_sys_autostart{4001};
 Subscription _sensor_gyro_fft_sub;
 bool _dynamic_notch_fft_available=false;
 uint32_t _selected_sensor_device_id=123;
 float _filter_sample_rate_hz=2000;
 Notch _dynamic_notch_filter_fft[3][3];
 int _dynamic_notch_filter_fft_update_perf=0,_dynamic_notch_filter_fft_disable_perf=0;
 void DisableDynamicNotchFFT(){_dynamic_notch_fft_available=false;for(auto &axis:_dynamic_notch_filter_fft)for(auto &n:axis)n.disable();}
 void UpdateDynamicNotchFFT(const hrt_abstime &,bool force=false);
};
//METHOD
int main(){
 int cases=0;
 for(int af:{4001,4081,4082,4014,5001,50000}) {
  for(float rate:{2000.f,8000.f}) {
   VehicleAngularVelocity v;v._param_sys_autostart.value=af;
   v._filter_sample_rate_hz=rate;v._sensor_gyro_fft_sub.data.sensor_sample_rate_hz=rate;
   v._sensor_gyro_fft_sub.data.resolution_hz=rate/512.f;
   v.UpdateDynamicNotchFFT(100);
   float expected=math::constrain(rate/512.f,8.f,30.f);
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
   if(af==4001 || af==4081 || af==4082 || af==4014)expected=20;
#endif
   assert(v._dynamic_notch_fft_available);
   for(auto &axis:v._dynamic_notch_filter_fft){assert(axis[0].freq==40);assert(axis[1].freq==85);assert(axis[2].freq==130);for(auto &n:axis)assert(fabsf(n.bw-expected)<0.001f);}
   v._sensor_gyro_fft_sub.data.peak_frequencies_y[1]=92;
   v.UpdateDynamicNotchFFT(200);assert(v._dynamic_notch_filter_fft[1][1].freq==92);cases++;
  }
 }
 for(int invalid=0;invalid<5;invalid++) {
  VehicleAngularVelocity v;
  if(invalid==0)v._sensor_gyro_fft_sub.data.device_id=456;
  if(invalid==1)v._sensor_gyro_fft_sub.data.sensor_sample_rate_hz=2300;
  if(invalid==2)v._sensor_gyro_fft_sub.valid=false;
  if(invalid==3)v._param_imu_gyro_dnf_en.value=0;
  v.UpdateDynamicNotchFFT(invalid==4?4000000:100);
  assert(!v._dynamic_notch_fft_available);
  for(auto &axis:v._dynamic_notch_filter_fft){for(auto &n:axis){assert(n.updates==0);}}
  cases++;
 }
 VehicleAngularVelocity v;v.UpdateDynamicNotchFFT(100);
 v._sensor_gyro_fft_sub.data.peak_frequencies_x[0]=0;
 v.UpdateDynamicNotchFFT(200);assert(v._dynamic_notch_filter_fft[0][0].freq==0);cases++;
#if defined(CONFIG_ARCH_BOARD_PX4_MYPX4_V1)
 // Same centers must update width when leaving/re-entering the scoped profile.
 VehicleAngularVelocity sw;sw.UpdateDynamicNotchFFT(100);
 sw._param_sys_autostart.value=5001;sw.UpdateDynamicNotchFFT(200);assert(sw._dynamic_notch_filter_fft[0][0].bw==8);
 sw._param_sys_autostart.value=4001;sw.UpdateDynamicNotchFFT(300);assert(sw._dynamic_notch_filter_fft[0][0].bw==20);cases++;
#endif
 printf("PASS: %d actual FFT-method scope/tracking/guard cases\n",cases);
}
'''
fixture = fixture.replace('#include <cstdio>', '#include <cstdio>\n#include <initializer_list>')
fixture = fixture.replace('//METHOD', method)
with tempfile.TemporaryDirectory() as d:
    test = Path(d) / 'test.cpp'
    test.write_text(fixture)
    for variant in ('mypx4', 'other_board'):
        exe = Path(d) / variant
        flags = ['-DCONFIG_ARCH_BOARD_PX4_MYPX4_V1'] if variant == 'mypx4' else []
        subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-Werror', *flags,
                        '-I'+str(root/'boards/px4/mypx4_v1/src'),str(test),'-o',str(exe)],check=True)
        print(variant, flush=True)
        subprocess.run([str(exe)],check=True)
