"""L2 maintenance-only serial service. Not part of the observation release.

Centering is opt-in. A durable backup must precede every calibration sequence.
The historical left-only firmware and the selected-arm firmware have different
torque policies: callers must supply the policy for their firmware.
"""
import datetime
import json
import os
import time


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
    if [r['id'] for r in rows]!=list(range(1,7)):raise ValueError('Expected selected arm IDs 1..6')
    for r in rows:
        raw=bytes.fromhex(r['eeprom_hex'])
        if not r['ok'] or len(raw)!=40 or raw[5]!=r['id'] or raw[33]!=0 or r['torque']!=0 or not 0<=r['position']<=4095:
            raise ValueError(f'Motor {r["id"]}: invalid readback, mode or torque')

def distance(a,b):
    d=abs(a-b);return min(d,4096-d)

def run(link,path,*,center=False,expected_firmware='left-calibration-v1',arm=None,verify_torque_only=False,reference_note='',before_center=None):
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
    if arm is not None:
        record['arm']=arm
        record['reference']='operator-positioned reference; arm and gripper pose recorded in reference_note'
    if arm is not None or reference_note:record['reference_note']=reference_note
    save(path,record) # Must succeed durably before ANY calibration write.
    if not center:return record
    if before_center is not None:
        try:
            record['centering_preflight']=before_center(second)
            save(path,record)
            if record['centering_preflight']['safe_to_center'] is not True:
                raise ValueError('2048 Home would cross the servo seam; see feasible_home_raw_interval in backup')
        except Exception as exc:
            record.update(status='preflight_rejected_no_calibration_writes',error=str(exc))
            save(path,record)
            raise
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
            if before_center is not None:
                # Small allowed hand drift still changes the proposed zero.
                # Recheck unmodified axes against the latest reading; use the
                # original evidence for axes already written in this sequence.
                remaining=[record['center_reference_rows'][str(row['id'])] if row['id'] in done else row
                           for row in live]
                record['centering_preflight']=before_center(remaining)
                if not record['centering_preflight']['safe_to_center']:
                    raise ValueError('Home drift moved a proposed limit across the servo seam')
            before=live[id-1];raw=bytes.fromhex(before['eeprom_hex']);offset=int.from_bytes(raw[31:33],'little')
            record.setdefault('center_reference_rows',{})[str(id)]=before
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
