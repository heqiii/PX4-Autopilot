"""Verify the committed MyPX4 4001 entry-profile package and startup contract."""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zlib


parser = argparse.ArgumentParser()
parser.add_argument("source", type=Path)
parser.add_argument("expected_git")
args = parser.parse_args()
root = args.source
build = root / "build/px4_mypx4_v1_default"
checks = []


def require(condition, label):
    assert condition, label
    checks.append(label)
    print("PASS " + label)


def code(text):
    return [line.strip() for line in text.splitlines()
            if line.strip() and not line.lstrip().startswith("#")]


package_path = build / "px4_mypx4_v1_default.px4"
binary_path = build / "px4_mypx4_v1_default.bin"
package = json.loads(package_path.read_text())
image = zlib.decompress(base64.b64decode(package["image"]))
binary = binary_path.read_bytes()
prototype = json.loads((root / "boards/px4/mypx4_v1/firmware.prototype").read_text())
require(package["git_hash"] == args.expected_git, "package Git matches committed source")
require(not subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"]),
        "firmware worktree clean")
for key in ("board_id", "board_name", "image_maxsize"):
    require(package[key] == prototype[key], f"{key} matches board prototype")
require(package["board_id"] == 53, "MyPX4 board ID 53")
require(0 < len(image) == package["image_size"] <= package["image_maxsize"],
        "image size and flash ceiling")
require(image == binary, "decoded package image equals BIN")
for topic in (b"manual_control_link_status", b"manual_control_input"):
    require(topic in image, f"diagnostic topic {topic.decode()} present")

params = ET.fromstring(zlib.decompress(base64.b64decode(package["parameter_xml"])))
marker = params.find(".//parameter[@name='MYPX4_CFG_VER']")
require(marker is not None and marker.findtext("max") == "26", "migration marker max 26")
for name in ("MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_VEL_MANUAL"):
    require(params.find(f".//parameter[@name='{name}']").findtext("min") == "0.3",
            f"{name} metadata permits 0.3")

airframes = ET.fromstring(zlib.decompress(base64.b64decode(package["airframe_xml"])))
ids = [el.attrib.get("id") for el in airframes.findall(".//airframe")]
require("4001" in ids and "4081" not in ids and "4082" not in ids,
        "4001 available and retired MyPX4 IDs absent")
require("50000" in ids and "51000" in ids, "other compiled vehicle profiles retained")

board_source = root / "boards/px4/mypx4_v1/init/rc.board_defaults"
board_generated = build / "etc/init.d/rc.board_defaults"
generic_source = root / "ROMFS/px4fmu_common/init.d/airframes/4001_quad_x"
generic_generated = build / "etc/init.d/airframes/4001_quad_x"
sensors_source = root / "boards/px4/mypx4_v1/init/rc.board_sensors"
sensors_generated = build / "etc/init.d/rc.board_sensors"
require(code(board_source.read_text()) == code(board_generated.read_text()),
        "generated board defaults match source")
require(code(sensors_source.read_text()) == code(sensors_generated.read_text()),
        "generated board sensors match source")
require(code(generic_source.read_text()) == code(generic_generated.read_text()),
        "generated 4001 airframe matches upstream source")
for path in ("airframes/50000_generic_rover_differential", "rc.rover_differential_defaults",
             "rc.rover_differential_apps"):
    source = root / "ROMFS/px4fmu_common/init.d" / path
    generated = build / "etc/init.d" / path
    require(code(source.read_text()) == code(generated.read_text()), "Rover startup script retained: " + path)
require(b'rover GNSS default failed' in image, 'Rover GNSS default branch linked')
require('CONFIG_MODULES_ROVER_DIFFERENTIAL=y' in (build/'boardconfig').read_text(),
        'differential Rover control compiled')
require(not (build / "etc/init.d/airframes/4081_mypx4_f450_x2212_skywalker").exists(),
        "4081 startup script absent from package input")


board = board_generated.read_text()
sensors = sensors_generated.read_text()
require('param mypx4-golden apply' in board, 'native transaction before autostart')
require('param mypx4-golden defaults' in sensors, 'golden defaults restored after airframe')
require('param set MYPX4_CFG_VER' not in sensors, 'old post-airframe migration removed')
require('exit 1' in board and 'exit 1' in sensors, 'failed sync/defaults stop startup')
require('set LOGGER_BUF 256' in board, '256 KiB logger buffer retained')
for command in ('bmi088 -A -R 4', 'bmi088 -G -R 4', 'icm42605 -R 4'):
    require(command in sensors, command+' rotation retained')
manifest=json.loads((root/'boards/px4/mypx4_v1/tests/golden_manifest.json').read_text())
header=(root/'boards/px4/mypx4_v1/src/mypx4_golden_values.hpp').read_text()
require(len(manifest['entries'])==959 and manifest['source_count']==963, '963 workbook source / 959 targets / 4 exceptions')
for e in manifest['entries']:
    require(f'{{"{e["name"]}", 0x{e["raw_u32"]:08x}u,' in header, e['name']+' source bits')
    metadata=params.find(f".//parameter[@name='{e['name']}']")
    require(metadata is not None and metadata.attrib['type']==('FLOAT' if e['type']=='REAL32' else 'INT32'), e['name']+' compiled type')
require(b'v1.0.15 golden hard sync' in image, 'v1.0.15 native transaction linked in image')
require('CONFIG_MODULES_GYRO_FFT=y' in (build/'boardconfig').read_text(), 'FFT module compiled')
require(next(e for e in manifest['entries'] if e['name']=='IMU_GYRO_DNF_EN')['raw_u32']==2, 'FFT dynamic notch bit enabled in board target')
require(next(e for e in manifest['entries'] if e['name']=='IMU_GYRO_FFT_EN')['raw_u32']==1, 'FFT analysis enabled in board target')
require(b'golden transaction failed' in image, 'transaction failure path linked in image')
require(b'revision 26 complete' in image, 'revision 26 completion linked')
require('CONFIG_UART5_BAUD=115200' in (build/'NuttX/nuttx/.config').read_text(), 'UART5 compiled initial baud 115200')
require('CONFIG_BOARD_SERIAL_TEL3="/dev/ttyS4"' in (root/'boards/px4/mypx4_v1/default.px4board').read_text(), 'TELEM3 maps to UART5 ttyS4')
require('param set-default SER_TEL3_BAUD 115200' in board and 'param set-default SER_TEL3_BAUD 57600' not in board,
        'board TELEM3 default 115200')
require(next(e for e in manifest['entries'] if e['name']=='SER_TEL3_BAUD')['raw_u32']==115200,
        'golden TELEM3 default 115200')
for name,bits in {'SER_TEL1_BAUD':115200,'MAV_0_FLOW_CTRL':0,'MAV_0_FORWARD':0,'MPC_ALT_MODE':1}.items():
    require(next(e for e in manifest['entries'] if e['name']==name)['raw_u32']==bits, name+' v1.0.15 target')
require('CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS6"' in (root/'boards/px4/mypx4_v1/default.px4board').read_text(), 'UART7 TELEM1 ttyS6 retained')
for name in ('MC_ROLLRATE_K','MC_PITCHRATE_K'):
    require(next(e for e in manifest['entries'] if e['name']==name)['raw_u32']==0x3f800000, name+' restored 1.00 target')
require(b'MyPX4 FFT20 candidate' in image, 'board-scoped FFT20 policy/status linked')
require(next(e for e in manifest['entries'] if e['name']=='MC_PITCHRATE_D')['raw_u32']==0x3b449ba6, 'restored pitch D0.003 target')
require(next(e for e in manifest['entries'] if e['name']=='MC_ROLLRATE_D')['raw_u32']==0x3b449ba6, 'roll D0.003 unchanged')

for name,bits in {'GPS_1_CONFIG':202,'GPS_1_PROTOCOL':6,'GPS_2_CONFIG':201,'SER_GPS2_BAUD':230400,'EKF2_GPS_CTRL':15,'GPS_YAW_OFFSET':0}.items():
    require(next(e for e in manifest['entries'] if e['name']==name)['raw_u32']==bits, name+' UM982 target')
require('CONFIG_BOARD_SERIAL_GPS2="/dev/ttyS7"' in (build/'boardconfig').read_text(), 'physical GPS2 UART8 ttyS7 mapping')
require('CONFIG_UART8_BAUD=230400' in (build/'NuttX/nuttx/.config').read_text(), 'UART8 initial baud230400')
require('CONFIG_DRIVERS_GPS=y' in (build/'boardconfig').read_text(), 'GPS driver compiled')
for name in (b'AGRICA', b'UNIHEADINGA', b'GPGGA'):
    require(name+b' COM2 0.05\r\n' in image, 'MyPX4 UM982 COM2 20 Hz request linked: '+name.decode())
for name in (b'AGRICA', b'UNIHEADINGA', b'GPGGA', b'GPRMC', b'GPGSA', b'GPGST'):
    require(name+b' COM2 0.2\r\n' in image, 'MyPX4 UM982 COM2 5 Hz request linked: '+name.decode())
require(all(b'UNIAGRICA COM' + port + b' 0.2' not in image for port in (b'1', b'2')),
        'obsolete velocity command absent')
require('GPS_1_CONFIG' in (build/'etc/init.d/rc.serial').read_text(), 'main GPS generated serial route retained')

for name,bits in {'GPS_2_PROTOCOL':1,'SER_GPS1_BAUD':0,'SENS_GPS_MASK':0,'SENS_GPS_PRIME':0,'EKF2_MAG_TYPE':1}.items():
    require(next(e for e in manifest['entries'] if e['name']==name)['raw_u32']==bits, name+' dual GNSS target')
require(b'MyPX4 dual GNSS selected' in image, 'board-scoped dual GNSS policy linked')
require(b'MyPX4 dual GNSS topic reservation failed' in image, 'independent cold-start publication path linked')
for name,bits in {'MC_ROLLRATE_P':0x3e19999a,'MC_PITCHRATE_P':0x3e19999a,'MC_ROLLRATE_I':0x3e4ccccd,'MC_PITCHRATE_I':0x3e4ccccd}.items():
    require(next(e for e in manifest['entries'] if e['name']==name)['raw_u32']==bits, name+' restored v1.0.9 target')
result = {
    'result':'PASS', 'checks':len(checks), 'git_head':args.expected_git,
    'board_id':package['board_id'], 'image_bytes':len(image),
    'image_max_bytes':package['image_maxsize'], 'configuration_migration_version':26,
    'px4_sha256':hashlib.sha256(package_path.read_bytes()).hexdigest(),
    'bin_sha256':hashlib.sha256(binary).hexdigest(), 'physical_validation':'NOT_PERFORMED',
}
print('RESULT '+json.dumps(result,ensure_ascii=False,sort_keys=True))
