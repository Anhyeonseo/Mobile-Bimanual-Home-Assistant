#!/usr/bin/env python3
"""Replay this installation's measured five-pose sequence only."""
import argparse,json,signal,sys,time
from pathlib import Path
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/so101_arm_bridge'))
from pose_locked_profile import compile_profile,STAGES,gripper_tolerance_urad
from teach_left_demo import load
from fixed_pick_demo import connection,TerminalCommands,OperatorStop,wait_command,save_json
from so101_arm_bridge.bimanual_stream_adapter import OperationalLimits,CANONICAL_JOINT_NAMES
from so101_arm_bridge.left_only_adapter import _LeftWireAdapter,LeftOnlyAdapter,LEFT_CAPABILITIES
from so101_arm_bridge.fixed_pick_demo import smooth_leg,execute_leg,OWNER,require_home
FIRMWARE=0x00024A07

def bundle(root=ROOT):
    capture=load(root/'config/left-five-poses.json')
    p=compile_profile(capture)
    if load(root/'config/profile.json')!=p:raise ValueError('저장 자세와 설치된 프로필이 다릅니다.')
    binding=load(root/'config/firmware-binding.json')
    if binding!={'firmware_version':FIRMWARE,'profile_sha256':p['profile_sha256'],'calibration_hash':p['firmware_calibration_hash']}:
        raise ValueError('설치된 펌웨어와 자세 파일의 식별값이 다릅니다.')
    return p

def limits_for(p):
    return OperationalLimits(CANONICAL_JOINT_NAMES[:6],tuple(b['minimum_urad'] for b in p['bounds']),tuple(b['maximum_urad'] for b in p['bounds']),p['profile_sha256'])
def targets_for(p):return [tuple(x/1e6 for x in q) for q in p['targets_urad']]

def compatible(h,p):
    return h.protocol_version==2 and h.joint_count==6 and h.firmware_version==FIRMWARE and h.capabilities==LEFT_CAPABILITIES and h.left_calibration_hash==h.right_calibration_hash==p['firmware_calibration_hash'] and not h.stop_latched

class TaughtWire(_LeftWireAdapter):
    firmware_version=FIRMWARE
    def __init__(self,tr,p,*,motion_authorized):
        self.calibration_hash=p['firmware_calibration_hash']
        # Current pose is validated against the installed bounds by the base
        # adapter; the first finite leg takes that measurement slowly to Home.
        super().__init__(tr,limits_for(p),home_reference=None,motion_authorized=motion_authorized)
        self._config=replace(self._config,terminal_settle_gripper_tolerance_urad=gripper_tolerance_urad(p))

class TaughtAdapter(LeftOnlyAdapter):
    def __init__(self,tr,p,*,motion_authorized=False):
        self._limits=limits_for(p);self._targets=targets_for(p);self._vertex=0
        self._wire=TaughtWire(tr,p,motion_authorized=motion_authorized)
    def start(self,owner,points,*,finite):
        if not finite or not points:raise ValueError('저장 자세 사이의 유한 구간만 재생할 수 있습니다.')
        matches=[i for i,q in enumerate(self._targets) if max(abs(a-b) for a,b in zip(points[-1].positions_rad,q,strict=True))<=.000001]
        allowed=[i for i in matches if abs(i-self._vertex)<=1]
        if len(allowed)!=1:raise ValueError('저장하지 않은 목표 또는 순서를 건너뛴 이동입니다.')
        target=allowed[0]
        if not self.heartbeat_required and target!=0:
            raise ValueError('첫 구간은 home 정렬이어야 합니다.')
        if self.heartbeat_required and max(abs(a-b) for a,b in zip(points[0].positions_rad,self._targets[self._vertex],strict=True))>.01:
            raise ValueError('현재 구간의 출발 자세가 다릅니다.')
        expected=smooth_leg(points[0].positions_rad,self._targets[target],self._limits)
        if len(points)!=len(expected) or any(a.offset_ms!=b.offset_ms or max(abs(x-y) for x,y in zip(a.positions_rad,b.positions_rad,strict=True))>.000001 for a,b in zip(points,expected)):
            raise ValueError('저장 경로의 속도 또는 보간 방식과 다릅니다.')
        result=super().start(owner,points,finite=True)
        self._vertex=target
        return result

def report(p):
    q=targets_for(p);limits=limits_for(p)
    current=0;legs=[]
    for target in (1,2,3,4,3,2,1,0):
        points=smooth_leg(q[current],q[target],limits)
        legs.append(dict(from_pose=STAGES[current],to_pose=STAGES[target],seconds=round(points[-1].offset_ms/1000,2)))
        current=target
    return dict(profile_sha256=p['profile_sha256'],legs=legs,firmware=hex(FIRMWARE),max_joint_speed_rad_s=.2,collision_checked=False,physical_replay_verified=False)

def play(args,p):
    if not args.execute:print(json.dumps(report(p),ensure_ascii=False,indent=2));return
    if not sys.stdin.isatty():raise ValueError('입력이 유지되는 대화형 젯슨 터미널에서 실행하세요.')
    limits=limits_for(p);targets=targets_for(p);commands=TerminalCommands(sys.stdin)
    log={'profile_sha256':p['profile_sha256'],'status':'STARTING','events':[],'physical_pick_verified':False}
    save_json(args.log,log)
    print('현재 자세에서 저속으로 Home에 이동한 뒤 집기를 시작합니다. 준비 중 토크가 해제됐다 켜지므로 왼팔과 물체를 받쳐주세요.',flush=True)
    print('들어 올린 뒤 유지: return + Enter로 내려놓기, home에서 run + Enter로 반복, q + Enter로 정지.',flush=True)
    with connection(args) as tr:
        adapter=TaughtAdapter(tr,p,motion_authorized=True);attempted=False;last=None
        def leg(index):
            nonlocal attempted,last
            print('이동:',STAGES[index],flush=True);attempted=True
            execute_leg(adapter,targets[index],limits,commanded_start=last,interrupt=commands.during_motion)
            last=targets[index];log['events'].append({'pose':STAGES[index],'reached':True})
        try:
            adapter.prepare()
            home=adapter.refresh_unarmed_anchor()
            # Fresh measured start, range/calibration checks and pre-ARM recheck.
            leg(0);sequence=(1,2,3,4)
            while True:
                for index in sequence:leg(index)
                location=sequence[-1];log['status']='HOLDING_'+STAGES[location].upper()
                print('유지 중: '+('return으로 내려놓기' if location==4 else 'run으로 다시 집기')+' / q로 종료',flush=True)
                while True:
                    cmd=wait_command(adapter,limits,targets[location],commands)
                    if cmd=='q':raise OperatorStop()
                    if cmd=='return' and location==4:sequence=(3,2,1,0);break
                    if cmd=='run' and location==0:sequence=(1,2,3,4);break
                    print('현재 위치에서 가능한 명령을 입력하세요.',flush=True)
        except OperatorStop:log['status']='STOPPED'
        except BaseException as e:
            log.update(status='FAILED',error=str(e) or type(e).__name__);raise
        finally:
            if attempted:
                try:adapter.stop(OWNER);log['stop_latched']=True
                except Exception as e:
                    log.update(stop_latched=False,stop_error=str(e));save_json(args.log,log)
                    raise RuntimeError('정지 응답 없음: 팔과 물체를 받치고 모터 전원을 차단하세요.') from e
            save_json(args.log,log)

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('command',choices=('inspect','check','play'))
    a.add_argument('--execute',action='store_true')
    a.add_argument('--port',default='/dev/ttyACM0')
    a.add_argument('--log',type=Path,default=ROOT/'output/last-run.json')
    args=a.parse_args();args.baud=921600;args.left_only=True;p=bundle()
    if args.command=='inspect':
        from dataclasses import asdict
        with connection(args) as tr:
            h=tr.enter_binary_mode();result=asdict(h);result['compatible']=compatible(h,p);result['motion_commands_sent']=False;print(json.dumps(result,indent=2))
            if not result['compatible']:raise ValueError('현재 제어기의 버전·자세 프로필·정지 상태가 다릅니다.')
    elif args.command=='check':print(json.dumps(report(p),ensure_ascii=False,indent=2))
    else:play(args,p)
if __name__=='__main__':
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGHUP,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:main()
    except (Exception,KeyboardInterrupt) as e:print('실행 중단:',e or '사용자 중단',file=sys.stderr);sys.exit(2)
