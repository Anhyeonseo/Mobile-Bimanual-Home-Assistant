from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'


def test_actual_observer_parser_and_reset_have_no_write_symbols(tmp_path):
    exe=tmp_path/'observer'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S2_OBSERVER_ONLY=1',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
        str(FAKE/'s2_observer_harness.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_transport_rejects_all_writes_on_both_buses(tmp_path):
    source=(FAKE/'left_calibration_gate.c').read_text()
    source=source.replace('assert(!ServoTransport_Register(&other));','assert(ServoTransport_Register(&other));')
    source=source.replace(' for(unsigned id=',' for(unsigned bus=0;bus<2;bus++)\n for(unsigned id=')
    source=source.replace('int allowed=id>=1&&id<=6&&address==40&&(value==0||value==128);','int allowed=0;')
    source=source.replace('ServoTransport_BeginService(&huart1,','ServoTransport_BeginService(bus?&other:&huart1,')
    source=source.replace('ServoTransport_Transmit(&huart1,','ServoTransport_Transmit(bus?&other:&huart1,')
    source=source.replace('ServoTransport_End(&huart1,','ServoTransport_End(bus?&other:&huart1,')
    path=tmp_path/'gate.c';path.write_text(source);exe=tmp_path/'gate'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S2_OBSERVER_ONLY=1',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(path),
        str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=30)
