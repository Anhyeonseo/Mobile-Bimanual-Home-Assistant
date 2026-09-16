import ctypes
import struct
import subprocess
from pathlib import Path
from types import SimpleNamespace
import pytest
from so101_arm_bridge.mobile_wire import crc32c, MobileClock
from so101_arm_bridge.robot_evidence import RobotEvidenceClient

ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def core(tmp_path):
    p=tmp_path/'wrapper.c';p.write_text('''#include "actuator_core/robot_evidence.h"
static actuator_robot_evidence_t state;
int accept(unsigned boot,unsigned age,unsigned now,const unsigned char *q,unsigned char *r){return actuator_robot_evidence_accept(&state,boot,age,now,q,r);}
int fresh(unsigned now,unsigned age){return actuator_robot_evidence_fresh(&state,now,age);}
''')
    lib=tmp_path/'evidence.so'
    subprocess.run(['gcc','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(ROOT/'firmware/stm32_actuator/include'),str(p),str(ROOT/'firmware/stm32_actuator/src/robot_evidence.c'),str(ROOT/'firmware/stm32_actuator/src/crc32c.c'),'-o',str(lib)],check=True)
    c=ctypes.CDLL(str(lib));c.accept.argtypes=[ctypes.c_uint32]*3+[ctypes.c_void_p]*2
    return c

def request(seq=1,boot=42,observed=1000,expires=1100,flags=7):
    q=bytearray(36);q[:4]=b'AE\1\1';struct.pack_into('<4I',q,4,seq,boot,observed,expires);q[20]=flags
    struct.pack_into('<I',q,32,crc32c(q[:32]));return bytes(q)

def exchange(core,q,now=1001):
    out=ctypes.create_string_buffer(64)
    ok=core.accept(42,100,now,q,out)
    return ok,out.raw


def test_real_c_accepts_fresh_zero_flags_and_expires_without_poll_refresh(core):
    ok,out=exchange(core,request());assert ok and out[:4]==b'AF\1\1'
    assert struct.unpack_from('<I',out,60)[0]==crc32c(out[:60])
    assert core.fresh(1099,100) and not core.fresh(1100,100)
    assert not exchange(core,request(),1002)[0]
    assert exchange(core,request(seq=2,observed=1002,expires=1102,flags=0),1003)[0]


@pytest.mark.parametrize('changes',({'boot':43},{'seq':0},{'observed':1002},{'observed':900},{'expires':1001},{'expires':1200},{'flags':8}))
def test_stale_future_wrong_boot_and_invalid_flags_never_admit(core,changes):
    assert not exchange(core,request(**changes))[0]
    assert not core.fresh(1001,100)


def test_timer_wrap_and_strict_reserved_bytes(core):
    assert exchange(core,request(observed=0xfffffff0,expires=84),now=5)[0]
    assert not core.fresh(84,100)
    q=bytearray(request(seq=2,observed=6,expires=90));q[24]=1
    struct.pack_into('<I',q,32,crc32c(q[:32]))
    assert not exchange(core,bytes(q),7)[0]


def test_python_client_preserves_sensor_age_through_actual_c_receiver(core):
    now=[1.05]
    clock=MobileClock();sync=SimpleNamespace(boot_id=42,mcu_tick_ms=1000)
    clock.sample=(1000,1002,sync)
    mobile=SimpleNamespace(clock=clock,boot_id=42,now=lambda:round(now[0]*1000),refresh_clock=lambda:None)
    packets=[]
    class Transport:
        def exchange_mobile(self,q):
            packets.append(q);ok,reply=exchange(core,q,round(now[0]*1000))
            assert ok;return reply
    client=RobotEvidenceClient(Transport(),mobile,lambda:now[0],maximum_age_ms=100)
    client.publish(1.02,arms_safe=True,lift_hold=True,payload_safe=False)
    packet=packets[0]
    assert struct.unpack_from('<I',packet,12)[0]<=1018 and packet[20]==3
    with pytest.raises(ValueError):client.publish(1.02,arms_safe=True,lift_hold=True,payload_safe=True)
    now[0]=1.2
    with pytest.raises(ValueError):client.publish(1.02,arms_safe=True,lift_hold=True,payload_safe=True)
    assert len(packets)==1
