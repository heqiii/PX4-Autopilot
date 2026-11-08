"""Run real generated serial scripts with command-recording host doubles.
No serial port is opened. Validates standalone main GPS on physical GPS2.
"""
from pathlib import Path
import argparse, json, shlex, subprocess, xml.etree.ElementTree as ET
p=argparse.ArgumentParser();p.add_argument('source',type=Path);a=p.parse_args()
b=a.source/'build/px4_mypx4_v1_default'
manifest=json.loads((a.source/'boards/px4/mypx4_v1/tests/golden_manifest.json').read_text())
values={e['name']:e['value'] for e in manifest['entries']}
params={e.attrib['name']:e.attrib['default'] for e in ET.parse(b/'parameters.xml').findall('.//parameter')}
params.update(values)
shell=r"""
function set() { printf -v "$1" "%s" "$2"; }
declare -A pv
function param() { [[ "$1" == compare && "${pv[$2]:-0}" == "$3" ]]; }
function gps() { echo "CALL gps $*"; }
function mavlink() { echo "CALL mavlink $*"; }
function uxrce_dds_client() { echo "CALL uxrce_dds_client $*"; }
function septentrio() { echo "CALL septentrio $*"; }
function crsf_rc() { echo "CALL crsf_rc $*"; }
function dsm_rc() { echo "CALL dsm_rc $*"; }
function ghst_rc() { echo "CALL ghst_rc $*"; }
function sbus_rc() { echo "CALL sbus_rc $*"; }
DUAL_GPS_ARGS=""
"""
for name,value in params.items():shell+='pv['+shlex.quote(name)+']='+shlex.quote(str(value))+'\n'
shell+='R='+shlex.quote(str(b)+'/')+'\n. '+shlex.quote(str(b/'etc/init.d/rc.serial'))+'\n'
r=subprocess.run(['bash'],input=shell,text=True,capture_output=True,check=True)
calls=[s for s in r.stdout.splitlines() if s.startswith('CALL ')]
gps=[s for s in calls if s.startswith('CALL gps ')]
assert gps==['CALL gps start -d /dev/ttyS7 -b p:SER_GPS2_BAUD -e /dev/ttyS0 -g p:SER_GPS1_BAUD'],gps
assert len([s for s in calls if '/dev/ttyS7' in s])==1,calls
assert 'Conflicting config' not in r.stdout,r.stdout
assert int(params['GPS_1_PROTOCOL'])==6 and int(params['SER_GPS2_BAUD'])==230400
assert int(params['GPS_2_PROTOCOL'])==1 and int(params['SER_GPS1_BAUD'])==0
assert int(params['SENS_GPS_MASK'])==0 and int(params['SENS_GPS_PRIME'])==0 and int(params['EKF2_MAG_TYPE'])==1
assert int(params['EKF2_GPS_CTRL'])==15 and float(params['GPS_YAW_OFFSET'])==0
assert int(params['SER_TEL3_BAUD'])==115200
assert len([c for c in calls if c.startswith('CALL mavlink start -d /dev/ttyS4 -b p:SER_TEL3_BAUD ')])==1,calls
print('PASS: actual generated rc.serial + rc.serial_port: UM982 main UART8/ttyS7 NMEA6/230400 + original GPS1/ttyS0 u-blox1/auto; no port conflicts')
for call in calls:print(call)
