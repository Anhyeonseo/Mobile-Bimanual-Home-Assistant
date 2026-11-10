"""Production READ start/poll/retirement + transport + parser + pair scheduler.

HAL/DMA hardware and clock are faked; lifecycle and ownership use production code. Unlike the
older tracking fixture, End really disables telemetry and leases are real.
"""
from pathlib import Path
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'


def function(text, signature):
    # Use definitions, not top-of-file forward declarations.
    start=text.index(signature)
    while text.index(';',start)<text.index('{',start):
        start=text.index(signature,start+len(signature))
    return text[start:text.index('\n}',start)+3]


@pytest.fixture(scope='module')
def harness(tmp_path_factory):
    d=tmp_path_factory.mktemp('feedback-integration')
    hal=(FAKE/'stm32g4xx_hal.h').read_text().replace('typedef struct { struct {',
        'typedef struct {uint32_t ErrorCode;} DMA;\ntypedef struct {uint32_t CR1,CR3;} REGS;\ntypedef struct { uint32_t ErrorCode; DMA *hdmarx; REGS *Instance; struct {')
    hal=hal.replace('#endif','''
#define ATOMIC_CLEAR_BIT(reg, bits) ((reg)&=~(bits))
#define USART_CR1_PEIE 1U
#define USART_CR1_RTOIE 2U
#define USART_CR3_EIE 4U
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
#define HAL_UART_ERROR_NONE 0U
#define HAL_DMA_ERROR_NONE 0U
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
#endif''')
    (d/'stm32g4xx_hal.h').write_text(hal)
    l=(BOARD/'Src/servo_bus.c').read_text();r=(BOARD/'Src/right_servo_bus.c').read_text()
    code=(FAKE/'feedback_recovery_harness.c').read_text()
    types=l[l.index('typedef enum\n'):l.index('static ServoInMotionTelemetry servo_in_motion')]
    types+=r[r.index('typedef enum\n'):r.index('static volatile uint8_t right_servo_rx_ring')]
    code=code.replace('/* PRODUCTION_TYPES */',types)
    funcs=[]
    for src,prefix in ((l,'Servo'),(r,'RightServoBus')):
        if prefix=='Servo':
            funcs.append(function(src,'static HAL_StatusTypeDef ServoInMotionTelemetry_Fail('))
            funcs.append(function(src,'static HAL_StatusTypeDef ServoBus_PrepareRuntimeRead('))
            funcs.append(function(src,'bool ServoBus_PrepareReader('))
        else:
            funcs.append(function(src,'static HAL_StatusTypeDef RightServo_TelemetryFail('))
            funcs.append(function(src,'static HAL_StatusTypeDef RightServoBus_InMotionTelemetryBeginOwned('))
        for sig in (f'HAL_StatusTypeDef {prefix}_InMotionTelemetryBegin(',f'void {prefix}_InMotionTelemetryEnd(',f'uint8_t {prefix}_InMotionTelemetryReleased(',
                    f'uint8_t {prefix}_InMotionTelemetryPending(',f'uint8_t {prefix}_InMotionTelemetryCanStart(',
                    f'{prefix}_InMotionTelemetryGetSnapshot(void)',
                    f'static HAL_StatusTypeDef {prefix}_InMotionTelemetryStartOwned(',
                    f'HAL_StatusTypeDef {prefix}_InMotionTelemetryStart(',
                    f'void {prefix}_InMotionTelemetryOnTxComplete(',
                    f'static HAL_StatusTypeDef {prefix}_InMotionTelemetryPollOwned(',
                    f'HAL_StatusTypeDef {prefix}_InMotionTelemetryPoll('):
            if 'GetSnapshot' in sig:
                sig=('const ServoInMotionTelemetrySnapshot *\n' if prefix=='Servo' else
                     'const RightServoInMotionTelemetrySnapshot *\n')+sig
            funcs.append(function(src,sig))
    code=code.replace('/* PRODUCTION_FUNCTIONS */','\n'.join(funcs))
    # Reinsert the exact old failure contract in a separate negative-control
    # binary. Its End is the real function and must make this test fail.
    for prefix in ('Servo','RightServoBus'):
        marker='    HAL_StatusTypeDef result = ServoFeedbackCompletion_Poll'
        start=code.index(f'HAL_StatusTypeDef {prefix}_InMotionTelemetryPoll(')
        at=code.index(marker,start)
        code=code[:at]+f"""
#ifdef REPRO_OLD_END_ON_TIMEOUT
    if (parsed == HAL_TIMEOUT) {{ {prefix}_InMotionTelemetryEnd(); return parsed; }}
#endif
"""+code[at:]
    (d/'harness.c').write_text(code)
    sources=[d/'harness.c',*[BOARD/'Src'/f'{n}.c' for n in (
        'servo_transport','bimanual_servo_dispatch','servo_joint_config','servo_rx_window','servo_response_parser','bimanual_tracking_feedback','s3_platform_snapshot')],
        *[CORE/'src'/f'{n}.c' for n in ('bimanual_dispatch','sts3215_packet','sts3215_response','bus_router','shared_bus','motor_groups')]]
    exe=d/'harness'
    compile_args=['gcc','-std=c11','-Wall','-Wextra','-Werror','-g',
        '-DHOST_S3_STATIONARY_PICK=1','-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1',
        '-I',str(d),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
        *map(str,sources),'-o',str(exe)]
    subprocess.run(compile_args,check=True)
    old=exe.with_name('old-contract')
    subprocess.run(compile_args[:-2]+['-DREPRO_OLD_END_ON_TIMEOUT','-o',str(old)],check=True)
    return exe


@pytest.mark.parametrize('bus',[0,1])
@pytest.mark.parametrize('slot',range(10))
@pytest.mark.parametrize('fault',[0,1,2,3,4,5,6,7,8,10,11,12,14,15])
def test_read_fault_recovery_and_hard_stop(harness,bus,slot,fault):
    subprocess.run([str(harness),str(bus),str(slot),str(fault)],check=True)


@pytest.mark.parametrize('bus',[0,1])
def test_minute_of_feedback_with_packet_loss_and_wraparound(harness,bus):
    subprocess.run([str(harness),str(bus),'0','9'],check=True)


def test_failure_wire_diagnostics_are_validated():
    import struct
    from so101_arm_bridge.stationary_telemetry import failure_request,decode_failures
    assert failure_request()==b'AF'+bytes(34)
    values=(1, 2,123,7,3,8,0,0,2, 0,0,0,0,0,0,0,0)
    result=decode_failures(b'\0'+struct.pack('<17I',*values))
    assert result['left']['first_tick_ms']==123 and result['left']['count']==2
    overflow=(1, 1,123,1,1,10<<16,0,0,2, 0,0,0,0,0,0,0,0)
    result=decode_failures(b'\0'+struct.pack('<17I',*overflow))
    assert result['left']['bus_failure_reason']==10 and result['left']['parser_reject']==0
    for payload in (b'',b'\0'*69,b'\1'+bytes(68)):
        with pytest.raises(ValueError):decode_failures(payload)


@pytest.mark.parametrize('bus',[0,1])
def test_old_end_on_timeout_contract_is_detected(harness,bus):
    result=subprocess.run([str(harness.with_name('old-contract')),str(bus),'2','1'],
                          capture_output=True,text=True)
    assert result.returncode != 0
    assert 'BimanualTrackingFeedback_CanStart()' in result.stderr


@pytest.mark.parametrize('bus',[0,1])
@pytest.mark.parametrize('scenario',[20,21,22,23,24,25])
def test_preparation_partial_start_stop_and_periodic_output(harness,bus,scenario):
    subprocess.run([str(harness),str(bus),'0',str(scenario)],check=True)


@pytest.mark.parametrize('bus',[0,1])
@pytest.mark.parametrize('step_us',[50,125,250])
def test_periodic_output_with_slow_main_loop_and_late_read_deferral(harness,bus,step_us):
    subprocess.run([str(harness),str(bus),str(step_us),'25'],check=True)
