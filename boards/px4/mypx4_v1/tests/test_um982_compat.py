from pathlib import Path
import argparse,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('source',type=Path);a=p.parse_args()
board=a.source/'boards/px4/mypx4_v1';gps=a.source/'src/drivers/gps/devices'
with tempfile.TemporaryDirectory() as d:
 exe=Path(d)/'nmea'
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wfloat-equal','-DGPS_DEFINITIONS_HEADER="nmea_host_definitions.h"',
  '-I'+str(board/'tests'),'-I'+str(gps/'src'),str(board/'tests/um982_compat_host.cpp'),
  *[str(gps/'src'/n) for n in ['nmea.cpp','gps_helper.cpp','unicore.cpp','rtcm.cpp','crc.cpp']],'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
 exe=Path(d)/'parser'
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(gps/'src'),str(gps/'gps-parser-test.cpp'),str(gps/'src/unicore.cpp'),str(gps/'src/crc.cpp'),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
