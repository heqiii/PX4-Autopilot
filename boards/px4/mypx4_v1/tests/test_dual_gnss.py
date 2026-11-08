"""Compile actual board policy and source methods with host backends, no hardware."""
from pathlib import Path
import argparse,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('source',type=Path);a=p.parse_args();root=a.source
board=root/'boards/px4/mypx4_v1'
def function(text,signature):
    start=text.index(signature);op=text.index('{',start);depth=1;end=op+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]
pub=(root/'platforms/common/uORB/PublicationMulti.hpp').read_text()
start=pub.index('template<typename T>');end=pub.index('/**\n * The publication multi class with data embedded.',start)
publication=pub[start:end]
gps=(root/'src/drivers/gps/gps.cpp').read_text()
publish=function(gps,'void\nGPS::publish()')
vehicle=(root/'src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp').read_text()
run=function(vehicle,'void VehicleGPSPosition::Run()')
fixture=(board/'tests/dual_gnss_host.cpp').read_text().replace('/*PUBLICATION_MULTI*/',publication).replace('/*GPS_PUBLISH*/',publish).replace('/*VEHICLE_GPS_RUN*/',run)
assert gps.index('reserve_primary_gps_topics')<gps.index('task_spawn(argc, argv, Instance::Secondary);')
assert 'data()' not in publish
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';cpp.write_text(fixture)
    for variant in ['mypx4','other_board']:
        exe=Path(d)/variant
        flags=['-DCONFIG_ARCH_BOARD_PX4_MYPX4_V1'] if variant=='mypx4' else []
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',*flags,'-I'+str(board/'src'),str(cpp),'-o',str(exe)],check=True)
        print(variant,flush=True);subprocess.run([str(exe)],check=True)
