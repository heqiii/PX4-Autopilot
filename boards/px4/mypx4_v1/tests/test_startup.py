"""Compile the actual transaction against fault-injecting host API doubles.

Persistence is modelled with PX4's float epsilon/default omission semantics.
This proves transaction behavior, not real FRAM power-loss durability.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('--generated', action='store_true')
a = p.parse_args()
board = a.source/'boards/px4/mypx4_v1'
build = a.source/'build/px4_mypx4_v1_default'
manifest = json.loads((board/'tests/golden_manifest.json').read_text())
metadata = ET.parse(build/'parameters.xml')
params = {}
for x in metadata.findall('.//parameter'):
    name, v, typ = x.attrib['name'], x.attrib['default'], x.attrib['type']
    params[name] = (struct.unpack('<I', struct.pack('<f', float(v)))[0] if typ == 'FLOAT' else int(v) & 0xffffffff, 9 if typ == 'FLOAT' else 6)
with tempfile.TemporaryDirectory() as d:
    tmp = Path(d)
    for h in ('px4_platform_common/px4_config.h', 'px4_platform_common/board_common.h', 'px4_platform_common/log.h', 'lib/parameters/param.h', 'uORB/Subscription.hpp', 'uORB/topics/vehicle_status.h'):
        q=tmp/h; q.parent.mkdir(parents=True, exist_ok=True); q.write_text('#pragma once\n')
    body = (board/'tests/golden_host.cpp').read_text()
    initial = '\n'.join('add('+json.dumps(k)+f',0x{v[0]:08x}u,{v[1]});' for k,v in params.items())
    expected = '\n'.join(f'assert(bits("{e["name"]}") == 0x{e["raw_u32"]:08x}u);' for e in manifest['entries'])
    body = body.replace('/*INITIAL*/',initial).replace('/*EXPECTED*/',expected)
    (tmp/'test.cpp').write_text(body)
    exe=tmp/'test'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(tmp),'-I'+str(board/'src'),str(tmp/'test.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

# Execute real NSH-compatible branches with command recording. The C++ logic
# above is tested separately; no hardware driver or serial port is opened.
init = build/'etc/init.d' if a.generated else board/'init'
defaults = init/'rc.board_defaults'
sensors = init/'rc.board_sensors'
assert 'revision 16' not in sensors.read_text()
shell = r"""
set -e
function set() { printf -v "$1" "%s" "$2"; }
function safety_button() { :; }
function param() {
 case "$1" in
  mypx4-golden) echo "GOLDEN $2"; return "$FAIL";;
  compare) [[ "$2" == 'SYS_AUTOSTART' && "$3" == "$AF" ]];;
  greater) return 1;;
  *) return 0;;
 esac
}
"""
for af, call in ((4001,'apply'), (4081,'apply'), (4082,'apply'), (4014,'apply'),
                 (50000,'defaults'), (51000,None), (5001,None)):
    for fail in (0,1) if af in (4001,50000) else (0,):
        sh = shell + f'AF={af} FAIL={fail}\n. "{defaults}"\necho CONTINUATION\n'
        r = subprocess.run(['bash'],input=sh,text=True,capture_output=True)
        actual = [line for line in r.stdout.splitlines() if line.startswith('GOLDEN ')]
        assert actual == ([f'GOLDEN {call}'] if call else []), (af,r.stdout)
        assert (r.returncode != 0) == bool(fail), (af,r.stderr)
        assert ('CONTINUATION' in r.stdout) != bool(fail), (af,r.stdout)
print('PASS: 7 startup routing cases + failed quad/Rover startup stops; generated='+str(a.generated))
