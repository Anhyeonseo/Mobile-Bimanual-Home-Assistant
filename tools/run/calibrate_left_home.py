#!/usr/bin/env python3
"""Temporary centering service: backup first, six explicit center writes, no motion."""
import argparse
import datetime
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/so101_arm_bridge'))

class Link:
    def __init__(self,port):self.port=port
    def exchange(self,command,*,scan=False):
        packet=(command+'\n').encode('ascii')
        if self.port.write(packet)!=len(packet):raise RuntimeError('Partial command write; do not retry centering')
        rows=[];deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            raw=self.port.readline()
            if not raw:continue
            value=json.loads(raw)
            if 'error' in value:raise RuntimeError(value['error'])
            if not scan:return value
            if value.get('scan_end'):
                if len(rows)!=6:raise RuntimeError('Incomplete six-motor scan')
                return rows
            rows.append(value)
        raise TimeoutError('Response uncertain; do not repeat CENTER')

def save(path,record):
    temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as f:
        json.dump(record,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)
    fd=os.open(path.parent,os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)

def validate(rows):
    if [r['id'] for r in rows]!=list(range(1,7)):raise ValueError('Expected left IDs 1..6')
    for r in rows:
        raw=bytes.fromhex(r['eeprom_hex'])
        if not r['ok'] or len(raw)!=40 or raw[5]!=r['id'] or raw[33]!=0 or r['torque']!=0 or not 0<=r['position']<=4095:
            raise ValueError(f'Motor {r["id"]}: invalid readback, mode or torque')

def distance(a,b):
    d=abs(a-b);return min(d,4096-d)

def run(link,path,*,center=False,expected_firmware='left-calibration-v1',arm=None,verify_torque_only=False,reference_note=''):
    if path.exists():raise FileExistsError('Use a new backup file; never overwrite previous calibration evidence')
    info=link.exchange('INFO')
    if info.get('firmware')!=expected_firmware or info.get('motion_enabled') is not False or info.get('ids')!=list(range(1,7)) or (arm is not None and info.get('arm')!=arm):
        raise RuntimeError('Wrong maintenance firmware')
    if center and info.get('consumed_mask')!=0:raise RuntimeError('Centering already attempted this boot; inspect existing record')
    if not verify_torque_only and link.exchange('OFF').get('off_ok') is not True:raise RuntimeError('Torque-off verification failed')
    first=link.exchange('SCAN',scan=True);validate(first)
    second=link.exchange('SCAN',scan=True);validate(second)
    for a,b in zip(first,second,strict=True):
        if a['eeprom_hex']!=b['eeprom_hex'] or distance(a['position'],b['position'])>8:raise RuntimeError('Home pose/configuration changed during backup')
    path.parent.mkdir(parents=True,exist_ok=True)
    record={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'firmware':info,'reference':'operator-positioned home; gripper middle','before':second,'results':[], 'motion_commands_sent':False,'status':'backup_saved','old_runtime_limits_valid':False}
    if arm is not None:record['arm']=arm
    if reference_note:record['reference_note']=reference_note
    save(path,record) # Must succeed durably before ANY calibration write.
    if not center:return record
    try:
        for id in (6,5,4,3,2,1):
            live=link.exchange('SCAN',scan=True);validate(live)
            done={r['id'] for r in record['results']}
            for row,original in zip(live,second,strict=True):
                if row['id'] not in done:
                    if row['eeprom_hex']!=original['eeprom_hex'] or distance(row['position'],original['position'])>8:
                        raise RuntimeError(f'Home moved before centering motor {row["id"]}')
                elif abs(row['position']-2048)>8:
                    raise RuntimeError('Already centered joint moved from Home')
            before=live[id-1];raw=bytes.fromhex(before['eeprom_hex']);offset=int.from_bytes(raw[31:33],'little')
            record.update(status='center_request_pending',pending_id=id);save(path,record)
            result=link.exchange(f'CENTER {id} {before["position"]} {offset}')
            record['results'].append(result);save(path,record)
            if result.get('id')!=id or result.get('center_ok') is not True or result.get('torque_off_verified') is not True:
                raise RuntimeError(f'Center failed or uncertain: {result}')
        final=link.exchange('SCAN',scan=True);validate(final)
        for old,new in zip(second,final,strict=True):
            a=bytes.fromhex(old['eeprom_hex']);b=bytes.fromhex(new['eeprom_hex'])
            if abs(new['position']-2048)>8 or any(a[i]!=b[i] for i in range(40) if i not in (31,32)):
                raise RuntimeError('Final pose/configuration verification failed')
        record.update(status='centered_verified',after=final,pending_id=None,power_cycle_persistence_verified=False)
        save(path,record);return record
    except BaseException as e:
        record.update(status='incomplete_do_not_retry',error=str(e))
        save(path,record)
        raise

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='/dev/ttyACM0')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--center',action='store_true',help='Write centering offsets at the supported, operator-set Home pose')
    args=p.parse_args()
    import serial
    from so101_arm_bridge.backend_lease import acquire_backend_lease
    from so101_arm_bridge.serial_port import open_exclusive_serial
    with acquire_backend_lease('stm32',0),open_exclusive_serial(serial,args.port,921600,.1) as port:
        port.reset_input_buffer()
        record=run(Link(port),args.output,center=args.center)
    print(json.dumps(record,indent=2))
if __name__=='__main__':main()
