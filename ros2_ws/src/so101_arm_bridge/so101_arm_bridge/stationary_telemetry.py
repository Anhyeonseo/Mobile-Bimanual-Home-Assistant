"""S3 read-only platform snapshot; raw single-turn values are not absolute height."""
import struct

def runtime_request():
    return b'AR'+bytes(34)

def decode_runtime(payload):
    if len(payload)!=37 or payload[0]!=0:
        raise ValueError('stationary runtime query rejected')
    version,flags,tick,line,fault_tick,fault_flags,dispatch,pairs,deferred=struct.unpack('<9I',payload[1:])
    if version!=1 or flags&~63 or fault_flags&~63:
        raise ValueError('stationary runtime format mismatch')
    return dict(firmware_tick_ms=tick,executor_active=bool(flags&1),settling=bool(flags&2),
                ready_for_stream=bool(flags&4),stop_latched=bool(flags&8),read_pending=bool(flags&16),
                idle_observer_active=bool(flags&32),first_stop_source_line=line,
                first_stop_tick_ms=fault_tick,first_stop_flags=fault_flags,
                dispatches_at_first_stop=dispatch,tracking_pairs_at_first_stop=pairs,
                read_window_deferrals=deferred)

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
