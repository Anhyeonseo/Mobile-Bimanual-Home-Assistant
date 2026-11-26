"""S3 read-only platform snapshot; raw single-turn values are not absolute height."""
import struct

def runtime_request():
    return b'AR'+bytes(34)

def decode_runtime(payload):
    if len(payload) not in (37,69,165,261) or payload[0]!=0:
        raise ValueError('stationary runtime query rejected')
    version,flags,tick,line,fault_tick,fault_flags,dispatch,pairs,deferred=struct.unpack('<9I',payload[1:37])
    if (version,len(payload)) not in ((1,37),(2,69),(3,165),(4,261)) or flags&~63 or fault_flags&~63:
        raise ValueError('stationary runtime format mismatch')
    result=dict(firmware_tick_ms=tick,executor_active=bool(flags&1),settling=bool(flags&2),
                ready_for_stream=bool(flags&4),stop_latched=bool(flags&8),read_pending=bool(flags&16),
                idle_observer_active=bool(flags&32),first_stop_source_line=line,
                first_stop_tick_ms=fault_tick,first_stop_flags=fault_flags,
                dispatches_at_first_stop=dispatch,tracking_pairs_at_first_stop=pairs,
                read_window_deferrals=deferred)
    if version>=2:
        keys=('fault','site','now_us','epoch_us','previous_poll_us','expired_jobs','schedule_faulted','bus_faulted')
        values=struct.unpack('<8I',payload[37:69])
        if values[0]>4 or values[1]>4 or values[6]>1 or values[7]>1:
            raise ValueError('mobile output diagnostic values invalid')
        result['mobile_output']=dict(zip(keys,values))
    if version>=3:
        keys=('source','tick_ms','reader_fault','lift_state','lift_fault','sample_ms','drive_sample_ms',
              'position_raw','velocity_raw','current_raw','moving','sample_flags','torque_enabled',
              'goal_velocity_raw','healthy','hold_verified','hold_first_ms','hold_last_ms','hold_anchor_raw',
              'hold_flags','interlock_ms','have_interlocks','previous_position_raw','height_um')
        values=list(struct.unpack('<24I',payload[69:165]))
        if values[0]>(6 if version>=4 else 5) or values[2]>7 or values[3]>6 or values[4]>5 or values[11]&~7 or values[19]&~3:
            raise ValueError('mobile board diagnostic values invalid')
        for i in (8,9,13,23):
            if values[i]&0x80000000:values[i]-=2**32
        result['mobile_board']=dict(zip(keys,values))
    if version>=4:
        keys=('failure_count','recovery_attempts','current_fault','sweep_mask',
              'first_fault','axis','motor_id','fields','request_ms','previous_sample_ms',
              'previous_temperature','previous_valid','reading_mode','data_length',
              'parser_terminal','motor_status','parser_reject')
        v=struct.unpack('<24I',payload[165:261])
        if (v[1]>3 or v[2]>7 or v[3]&~15 or v[4]>7 or v[5]>3 or v[7]&~127
                or v[11]>1 or v[12]>1 or v[13]>16 or v[14]>3 or v[16]>4 or v[23]>1):
            raise ValueError('mobile feedback diagnostic values invalid')
        fault=dict(zip(keys,v));body=struct.pack('<5I',*v[17:22])[:18]
        fault['raw_body_hex']=body.hex()
        fault['previous_drive_sample_ms']=v[22];fault['previous_drive_valid']=bool(v[23])
        fault['sample_gap_ms']=(v[8]-v[9])&0xffffffff if v[11] else None
        fault['reported_temperature']=body[8] if not v[12] and v[13]==15 and v[14]==1 else None
        if v[7]==8 and not v[12] and v[13]==1 and v[14]==1:
            fault['reported_temperature']=body[1]
            fault['temperature_register_confirmed']=True
        result['mobile_feedback']=fault
    return result

def request(operation=0):
    if type(operation) is not int or operation not in (0,1,2):
        raise ValueError('status/start idle reads/suspend idle reads only')
    return b'AP'+bytes((operation,))+bytes(33)

def decode(payload):
    if len(payload)!=45 or payload[0]!=0:
        raise ValueError('stationary telemetry request rejected')
    version,active,mask,reserved,tick,count,*values=struct.unpack('<BBBBII8I',payload[1:])
    if version!=1 or active not in (0,1) or mask&~15 or reserved:
        raise ValueError('stationary telemetry format mismatch')
    if any(v>4095 for v in values[:4]):
        raise ValueError('platform position outside single-turn encoder range')
    return dict(active=bool(active),present_mask=mask,firmware_tick_ms=tick,
                sequence=count,positions_raw=tuple(values[:4]),sample_age_ms=tuple(values[4:]))


def failure_request():
    return b'AF'+bytes(34)

def decode_failures(payload):
    if len(payload)!=69 or payload[0]!=0:
        raise ValueError('stationary feedback failure query rejected')
    values=struct.unpack('<17I',payload[1:])
    if values[0]!=1:
        raise ValueError('stationary feedback failure format mismatch')
    keys=('count','first_tick_ms','slot','hal_status','parser_status','uart_errors','dma_errors','phase')
    result={name: dict(zip(keys,values[start:start+8])) for name,start in (('left',1),('right',9))}
    for record in result.values():
        if record['count'] and (record['slot']>9 or record['hal_status'] not in (1,3) or record['phase'] not in (1,2,3)):
            raise ValueError('stationary feedback failure values invalid')
        # Firmware 0x24911 preserves the left bus reason in previously unused
        # high bits. Old records/right bus may leave this zero (unavailable).
        record['bus_failure_reason']=record['parser_status']>>16
        record['parser_reject']=record['parser_status']&255
        record['motor_status']=(record['parser_status']>>8)&255
    return result


def describe_runtime_fault(runtime):
    """Observed rejection, not a claim that physical overheating occurred."""
    f=runtime.get('mobile_feedback',{})
    if f.get('failure_count'):
        reason=[]
        for bit,label in ((1,'모드'),(2,'토크'),(4,'전압'),(8,'온도'),(16,'전류'),(32,'움직임 표시'),(64,'부하 응답')):
            if f['fields']&bit:reason.append(label)
        detail='·'.join(reason) or {1:'응답 지연',2:'장치 오류 응답',3:'응답 길이',4:'중복 응답',7:'통신 오류'}.get(f['first_fault'],'상태 조회')
        if f.get('reported_temperature') is not None and f['fields']&8:
            detail+=f" {f['reported_temperature']}°C 표시"
            if f.get('previous_valid'):
                detail+=f" (이전 {f['previous_temperature']}°C, {f['sample_gap_ms']}ms 전)"
        return f"{f['motor_id']}번 모터 {detail}로 이동 차단 · 최초 오류 기록 · 자동 재출발 없음"
    if runtime.get('mobile_board',{}).get('source')==1:
        return '자세 상태 갱신 지연으로 정지 · 자동 재출발 없음'
    return '제어기 정지 · 상태 확인 중 · 자동 재출발 없음'
