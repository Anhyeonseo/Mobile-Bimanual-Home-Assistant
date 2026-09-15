import ctypes
from dataclasses import replace
from pathlib import Path
import subprocess
import pytest
from so101_arm_bridge.mobile_wire import MobileCommand, crc32c

ROOT=Path(__file__).resolve().parents[1]
class CMessage(ctypes.Structure):
    _fields_=[('opcode',ctypes.c_int),('session',ctypes.c_uint32),('sequence',ctypes.c_uint32),
              ('valid_until_ms',ctypes.c_uint32),('velocity_raw',ctypes.c_int32*4)]

@pytest.fixture(scope='module')
def codec(tmp_path_factory):
    path=tmp_path_factory.mktemp('mobile-codec')/'codec.so'
    core=ROOT/'firmware/stm32_actuator'
    subprocess.run(['gcc','-std=c11','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(core/'include'),
                    *(str(core/'src'/f) for f in ('mobile_wire.c','mobile_supervisor.c','crc32c.c')),'-o',str(path)],check=True)
    lib=ctypes.CDLL(str(path))
    lib.actuator_mobile_wire_encode.argtypes=[ctypes.POINTER(CMessage),ctypes.POINTER(ctypes.c_uint8)]
    lib.actuator_mobile_wire_encode.restype=ctypes.c_bool
    lib.actuator_mobile_wire_decode.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_size_t,ctypes.POINTER(CMessage)]
    lib.actuator_mobile_wire_decode.restype=ctypes.c_bool
    return lib

@pytest.mark.parametrize('command',[
    MobileCommand(1,1,0,100), MobileCommand(3,2,10,200),
    MobileCommand(2,3,0xffffffff,4,(-32767,0,32767,-1)),
    MobileCommand(2,0xffffffff,0,0,(1,-1,2,-2))])
def test_python_and_compiled_c_exchange_identical_frames(codec,command):
    frame=command.encode()
    message=CMessage(command.opcode,command.session,command.sequence,command.valid_until_ms,(ctypes.c_int32*4)(*command.velocity_raw))
    output=(ctypes.c_uint8*36)()
    assert codec.actuator_mobile_wire_encode(ctypes.byref(message),output)
    assert bytes(output)==frame
    decoded=CMessage()
    assert codec.actuator_mobile_wire_decode(output,36,ctypes.byref(decoded))
    assert tuple(decoded.velocity_raw)==command.velocity_raw
    assert MobileCommand.decode(bytes(output))==command


def test_crc_has_independent_standard_check_vector():
    assert crc32c(b'123456789')==0xe3069283

@pytest.mark.parametrize('kwargs',[{'session':0},{'sequence':True},{'opcode':4},{'valid_until_ms':-1},
                                   {'velocity_raw':(32768,0,0,0)},{'velocity_raw':(0,0,0)},{'opcode':1,'sequence':1}])
def test_invalid_fields_rejected(kwargs):
    with pytest.raises(ValueError): replace(MobileCommand(2,1,0,10),**kwargs).encode()


def test_corruption_and_truncation_rejected_in_both_languages(codec):
    frame=MobileCommand(2,1,1,10,(1,2,3,4)).encode()
    for i in range(36):
        damaged=bytearray(frame);damaged[i]^=1
        with pytest.raises(ValueError): MobileCommand.decode(bytes(damaged))
        assert not codec.actuator_mobile_wire_decode((ctypes.c_uint8*36).from_buffer_copy(damaged),36,ctypes.byref(CMessage()))
    for length in range(36):
        with pytest.raises(ValueError): MobileCommand.decode(frame[:length])
