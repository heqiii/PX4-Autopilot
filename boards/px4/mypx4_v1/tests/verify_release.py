"""Verify the maintenance release's package and unchanged hardware/control baseline."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zlib

BASE = "30cc1d2a4f66b1c4f6a9880142b3a4e4674277b9"
ap = argparse.ArgumentParser()
ap.add_argument("source", type=Path)
ap.add_argument("expected_git")
args = ap.parse_args()
root = args.source
build = root / "build/px4_mypx4_v1_default"
checks = []

def require(value, label):
    assert value, label
    checks.append(label)
    print("PASS " + label)

def baseline(path):
    return subprocess.check_output(["git", "-C", str(root), "show", f"{BASE}:{path}"])

def commands(text):
    return [line.strip() for line in text.splitlines() if line.strip().startswith("param set-default ")]

pkg_path = build / "px4_mypx4_v1_default.px4"
bin_path = build / "px4_mypx4_v1_default.bin"
pkg = json.loads(pkg_path.read_text())
image = zlib.decompress(base64.b64decode(pkg["image"]))
proto = json.loads((root / "boards/px4/mypx4_v1/firmware.prototype").read_text())
require(pkg["git_hash"] == args.expected_git, "package Git identity equals requested commit")
for key in ("board_id", "board_name", "image_maxsize"):
    require(pkg[key] == proto[key], f"package {key} matches board prototype")
require(image == bin_path.read_bytes(), "decoded package image equals BIN")
require(0 < len(image) == pkg["image_size"] <= pkg["image_maxsize"], "image fits flash and size metadata")
require(not subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"]), "source worktree clean")
params = ET.fromstring(zlib.decompress(base64.b64decode(pkg["parameter_xml"])))
marker = params.find(".//parameter[@name='MYPX4_CFG_VER']")
require(marker is not None and marker.findtext("max") == "7", "packaged configuration revision max is 7")
ids = [el.attrib.get("id") for el in ET.fromstring(zlib.decompress(base64.b64decode(pkg["airframe_xml"]))).findall(".//airframe")]
require(ids.count("4081") == 1 and "4082" not in ids, "custom 4081 unique and retired 4082 absent")
require("50000" in ids and "51000" in ids, "differential and Ackermann metadata available")

unchanged = ["boards/px4/mypx4_v1/default.px4board",
             "boards/px4/mypx4_v1/firmware.prototype",
             "boards/px4/mypx4_v1/nuttx-config/nsh/defconfig",
             "boards/px4/mypx4_v1/src/board_config.h",
             "boards/px4/mypx4_v1/src/init.cpp", "boards/px4/mypx4_v1/src/spi.cpp",
             "boards/px4/mypx4_v1/src/i2c.cpp", "boards/px4/mypx4_v1/src/mtd.cpp",
             "boards/px4/mypx4_v1/src/timer_config.cpp",
             "boards/px4/mypx4_v1/init/rc.board_sensors", "src/modules/logger/logged_topics.cpp"]
for path in unchanged:
    require((root / path).read_bytes() == baseline(path), f"unchanged baseline: {path}")

for source_path, generated_path in [
    ("boards/px4/mypx4_v1/init/rc.board_defaults", "etc/init.d/rc.board_defaults"),
    ("ROMFS/px4fmu_common/init.d/airframes/4081_mypx4_f450_x2212_skywalker", "etc/init.d/airframes/4081_mypx4_f450_x2212_skywalker"),
]:
    current = (root / source_path).read_text()
    generated = (build / generated_path).read_text()
    require(commands(current) == commands(baseline(source_path).decode()), f"all ordered defaults preserved: {source_path}")
    # ROMFS generation strips comments; compare executable non-empty lines.
    code = lambda text: [line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    require(code(current) == code(generated), f"generated startup equals source: {generated_path}")

board = (build / "etc/init.d/rc.board_defaults").read_text()
require("set LOGGER_BUF 256" in board, "logger buffer remains 256 KiB")
linked = (build / "px4_mypx4_v1_default.map").read_text()
for module in ("gyro_fft", "rover_ackermann", "rover_differential", "mc_rate_control", "mc_att_control", "mc_pos_control"):
    require("modules__" + module in linked, "module linked: " + module)
result = {"result": "PASS", "checks": len(checks), "git_head": pkg["git_hash"],
          "baseline_git": BASE, "image_bytes": len(image), "image_max_bytes": pkg["image_maxsize"],
          "px4_sha256": hashlib.sha256(pkg_path.read_bytes()).hexdigest(),
          "bin_sha256": hashlib.sha256(bin_path.read_bytes()).hexdigest(),
          "configuration_migration_version": 7, "physical_validation": "NOT_PERFORMED"}
print("RESULT " + json.dumps(result, sort_keys=True))
