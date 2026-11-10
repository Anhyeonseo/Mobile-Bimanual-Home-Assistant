from dataclasses import replace
from types import SimpleNamespace as NS
import struct

import cv2
import numpy as np
import pytest

from home_robot_tasks.carry_rgbd import SurfacePatchTracker,StationarySceneCheck
from home_robot_tasks.carry_live_inputs import joint_exposure,match_exposure,FixedPlatformGuard,MeasuredFK,LiveCarryInputs
from so101_arm_bridge.stationary_telemetry import request,decode
from so101_arm_bridge.stationary_runtime import ResidentStop
from test_carry_execution import program


def test_runtime_status_reports_explicit_readiness_and_original_stop():
    from so101_arm_bridge.stationary_telemetry import runtime_request,decode_runtime
    assert runtime_request()==b'AR'+bytes(34)
    p=b'\x00'+struct.pack('<9I',1,36,90000,3201,89000,17,1,0,50)
    status=decode_runtime(p)
    assert status['ready_for_stream'] and status['idle_observer_active']
    assert not status['stop_latched']
    assert status['first_stop_source_line']==3201 and status['first_stop_flags']==17
    assert status['dispatches_at_first_stop']==1 and status['read_window_deferrals']==50
    for invalid in (p[:-1],b'\x01'+p[1:],b'\x00'+struct.pack('<9I',2,0,0,0,0,0,0,0,0),
                    b'\x00'+struct.pack('<9I',1,64,0,0,0,0,0,0,0)):
        with pytest.raises(ValueError):decode_runtime(invalid)


def scene():
    rng=np.random.default_rng(9)
    color=rng.integers(40,220,(160,240,3),dtype=np.uint8)
    depth=np.ones((160,240));mask=np.zeros(depth.shape,bool);mask[40:120,80:160]=True
    intr=dict(fx=300.,fy=300.,ppx=120.,ppy=80.,model='distortion.inverse_brown_conrady',coeffs=[0.]*5)
    return color,depth,mask,intr


def tracker():
    color,depth,mask,intr=scene()
    return SurfacePatchTracker(color,depth,mask,intr,np.eye(4).tolist(),patch_id='a',observed_s=10.,delta_error_floor_m=.0006)


def test_fixed_feature_patch_tracks_image_translation_without_centroid_switch():
    color,depth,mask,intr=scene();t=tracker()
    moved=cv2.warpAffine(color,np.array([[1.,0,2],[0,1.,0]],np.float32),(240,160))
    sample=t.update(moved,depth,10.033)
    assert sample.xyz_base_m[0]-t.initial.xyz_base_m[0]==pytest.approx(2/300,abs=.0002)
    assert sample.patch_id==t.initial.patch_id
    assert sample.sequence==1


@pytest.mark.parametrize('failure',['stale','holes','shape','occlusion'])
def test_lost_patch_never_silently_redetects(failure):
    color,depth,_,_=scene();t=tracker();stamp=10.033
    if failure=='stale':stamp=11.
    if failure=='holes':depth[:]=0
    if failure=='shape':color=color[:100]
    if failure=='occlusion':color[:]=0
    with pytest.raises(ValueError):t.update(color,depth,stamp)
    color,depth,_,_=scene()
    with pytest.raises(ValueError,match='LOST'):t.update(color,depth,10.05)


def test_scene_revalidation_detects_target_and_background_changes():
    color,depth,mask,_=scene();s=StationarySceneCheck(color,depth,mask)
    assert s.compare(color,depth)['unchanged']
    changed=depth.copy();changed[mask]+=.02
    assert not s.compare(color,changed)['unchanged']
    assert s.compare(color,changed,background_only=True)['unchanged']
    changed=depth.copy();changed[~mask]+=.02
    assert not s.compare(color,changed,background_only=True)['unchanged']


def test_depth_guard_ignores_only_far_geometry_noise_not_rgb_or_workspace_changes():
    color,depth,mask,_=scene()
    depth[:30]=1.8
    s=StationarySceneCheck(color,depth,mask,maximum_background_depth_m=1.2)
    changed=depth.copy();changed[:30]+=.03
    assert s.compare(color,changed)['unchanged']
    # Even a small far-to-near entrant must not disappear in a whole-image percentile.
    changed[4:10,4:10]=.8
    result=s.compare(color,changed)
    assert not result['unchanged']
    assert result['regions']['background']['entering_workspace_pixels']==36
    changed=depth.copy();changed[30:,:]=1.02
    assert not s.compare(color,changed)['unchanged']
    changed=depth.copy();changed[mask]+=.01
    assert not s.compare(color,changed)['unchanged']
    recolored=color.copy();recolored[:30]=0
    assert not s.compare(recolored,depth)['unchanged']


def test_depth_guard_preserves_missing_workspace_depth_rejection():
    color,depth,mask,_=scene()
    s=StationarySceneCheck(color,depth,mask,maximum_background_depth_m=1.2)
    changed=depth.copy();changed[~mask]=0
    assert not s.compare(color,changed)['unchanged']


def test_background_removal_preserves_the_more_occupied_planned_scene():
    color,depth,mask,_=scene()
    check=StationarySceneCheck(color,depth,mask,maximum_background_depth_m=1.2)
    removed=depth.copy();removed[:30]=1.15
    result=check.compare(color,removed)
    assert result['unchanged']
    assert result['regions']['background']['depth_difference95_m']>.1
    # Never relearn or mutate the originally checked scene from this observation.
    assert np.array_equal(check.depth,depth)
    moved_target=removed.copy();moved_target[mask]+=.02
    assert not check.compare(color,moved_target)['unchanged']


def test_disconnected_depth_speckles_do_not_combine_into_one_obstacle():
    color,depth,mask,_=scene();depth[:30]=1.8
    check=StationarySceneCheck(color,depth,mask,maximum_background_depth_m=1.2)
    changed=depth.copy();changed[5,::4]=.8
    result=check.compare(color,changed)
    assert result['regions']['background']['entering_workspace_pixels']==60
    assert result['unchanged']
    assert result['regions']['background']['largest_entering_component_pixels']==1


@pytest.mark.parametrize('old_depth',[1.,1.8])
def test_small_connected_nearer_obstacle_is_rejected_inside_or_outside_workspace(old_depth):
    color,depth,mask,_=scene();depth[:30]=old_depth
    check=StationarySceneCheck(color,depth,mask,maximum_background_depth_m=1.2)
    changed=depth.copy();changed[4:10,4:10]=.8
    result=check.compare(color,changed)
    assert not result['unchanged']
    assert result['regions']['background']['entering_workspace_pixels']==36
    # The same criterion retains thin diagonal structures; no erosion/opening.
    changed=depth.copy()
    for i in range(30):changed[i,i]=.8
    assert not check.compare(color,changed)['unchanged']


def test_scene_failure_preserves_region_diagnostics_for_operator():
    from home_robot_tasks.carry_rgbd import SceneValidationError
    color,depth,mask,intr=scene()
    inputs=LiveCarryInputs(program(),lambda q:np.eye(4),(color,depth),mask,intr,
                          np.eye(4).tolist(),FixedPlatformGuard(.3,.00002),clock=lambda:10.04)
    inputs.add_joints(packet(),10.03,10.034)
    inputs.add_frame({'color_time_s':10.025,'depth_time_s':10.026},
                     (color,((depth+.02)*1000).astype(np.uint16),{'depth_scale_m':.001}))
    with pytest.raises(SceneValidationError) as error:inputs.feedback()
    assert error.value.diagnostics['regions']['target']['depth_difference95_m']==pytest.approx(.02)


def packet(**kw):
    return NS(**(dict(status_code=0,joint_count=12,present_mask=4095,positions_urad=(0,)*12,
        sample_age_ms=(30,25,20,15,10,5)*2,completed_pairs=10)|kw))


def test_joint_age_and_round_trip_bound_are_preserved():
    j=joint_exposure(packet(),10.,10.004)
    assert j.lower_s==pytest.approx(9.970)
    assert j.upper_s==pytest.approx(9.999)
    assert match_exposure([j],10.005,10.006,10.02)==j
    with pytest.raises(ValueError):match_exposure([j],10.021,10.022,10.03)
    with pytest.raises(ValueError):joint_exposure(packet(),10.,10.021)
    with pytest.raises(ValueError):joint_exposure(packet(sample_age_ms=(151,)*12),10.,10.001)


def platform(tick=1000,raw=(0,0,0,4095),**kw):
    return dict(active=True,present_mask=15,firmware_tick_ms=tick,sequence=20,positions_raw=raw,sample_age_ms=(100,70,35,0))|kw


def test_platform_raw_wrap_is_relative_only_and_restart_is_rejected():
    g=FixedPlatformGuard(.3,-.00002);g.update(platform(),10,10.001)
    g.update(platform(1030,(0,0,0,0)),10.03,10.031)
    assert g.current_height==pytest.approx(.29998)
    with pytest.raises(ValueError,match='restarted'):g.update(platform(10),10.04,10.041)


@pytest.mark.parametrize('pause_s',[.7,1.01,2.,5.])
def test_known_configuration_gap_uses_clock_agreement_not_a_one_second_cutoff(pause_s):
    g=FixedPlatformGuard(.5,-.00002);g.update(platform(),10.,10.001)
    origin=g.origin.copy()
    g.update(platform(1000+round(pause_s*1000)),10.+pause_s,10.001+pause_s)
    assert np.array_equal(g.origin,origin) and g.current_height==.5
    assert g.stamp==pytest.approx(9.9+pause_s)  # Real oldest exposure, not arrival.


@pytest.mark.parametrize('tick',[1000,1100,5000,10])
def test_setup_pause_still_rejects_a_reset_or_paused_controller_clock(tick):
    g=FixedPlatformGuard(.5,-.00002);g.update(platform(),10.,10.001)
    with pytest.raises(ValueError,match='clock discontinuity'):
        g.update(platform(tick),12.,12.001)
    assert g.tick==1000 and g.stamp==pytest.approx(9.9)


def test_clock_wrap_and_motion_after_setup_keep_original_height_reference():
    g=FixedPlatformGuard(.5,-.00002);g.update(platform(0xffffffff-499),10.,10.001)
    g.update(platform(500),11.,11.001)
    with pytest.raises(ValueError,match='moved'):
        g.update(platform(1500,(0,0,0,15)),12.,12.001)


def test_platform_accepts_actual_profile_at_mechanical_bottom():
    import json
    from pathlib import Path
    profile=json.loads((Path(__file__).resolve().parents[1]/'config/home_assistant/s2_coordinates.development.json').read_text())
    scale=profile['lift']['meters_per_count']
    assert scale<0
    g=FixedPlatformGuard(0.,scale)
    g.update(platform(raw=(0,0,0,100)),10,10.001)
    g.update(platform(1030,(0,0,0,99)),10.03,10.031)
    assert g.current_height==pytest.approx(-scale)
    assert g.base_stopped and g.lift_stopped


@pytest.mark.parametrize('scale',[-.00002,.00002])
def test_platform_height_uses_profile_direction_once(scale):
    g=FixedPlatformGuard(.3,scale)
    g.update(platform(raw=(0,0,0,100)),10,10.001)
    g.update(platform(1030,(0,0,0,103)),10.03,10.031)
    assert g.current_height==pytest.approx(.3+3*scale)


@pytest.mark.parametrize('height,scale',[(-.01,-.00002),(float('nan'),-.00002),(0.,0.),(0.,float('nan')),(0.,float('inf'))])
def test_platform_invalid_height_or_scale_is_rejected(height,scale):
    with pytest.raises(ValueError):FixedPlatformGuard(height,scale)


@pytest.mark.parametrize('change',[{'positions_raw':(10,0,0,4095)}, {'positions_raw':(0,0,0,50)},
    {'present_mask':7},{'sample_age_ms':(201,0,0,0)}, {'active':False}])
def test_platform_guard_rejects_movement_missing_or_stale(change):
    g=FixedPlatformGuard(.3,.00002);g.update(platform(),10,10.001)
    with pytest.raises(ValueError):g.update(platform(1030,**change),10.03,10.031)


def test_platform_reply_delay_consumes_age_budget_without_restamping():
    from home_robot_tasks.carry_live_inputs import ObservationNotFresh
    g=FixedPlatformGuard(.3,.00002)
    g.update(platform(),10.,10.04)  # 40 ms reply plus 100 ms oldest sample.
    assert g.stamp==pytest.approx(9.9)
    with pytest.raises(ObservationNotFresh):g.update(platform(1040),10.04,10.15)
    assert g.stamp==pytest.approx(9.9) and g.tick==1000
    assert g.last_query['round_trip_ms']==pytest.approx(110)


def test_late_joint_reply_does_not_replace_last_accepted_sample():
    from home_robot_tasks.carry_live_inputs import ObservationNotFresh
    color,depth,mask,intr=scene();now=[10.04]
    guard=FixedPlatformGuard(.3,.00002);guard.update(platform(),10.03,10.034)
    inputs=LiveCarryInputs(program(),lambda q:np.eye(4),(color,depth),mask,intr,np.eye(4).tolist(),guard,clock=lambda:now[0])
    inputs.add_joints(packet(),10.03,10.034)
    inputs.add_frame({'color_time_s':10.025,'depth_time_s':10.026},
                     (color,(depth*1000).astype(np.uint16),{'depth_scale_m':.001}))
    original=inputs.feedback()
    with pytest.raises(ObservationNotFresh):inputs.add_joints(packet(completed_pairs=11),10.04,10.07)
    assert inputs.feedback()==original
    now[0]=10.3
    with pytest.raises(ValueError,match='stale'):inputs.feedback()


def test_platform_wire_enforces_read_only_operations():
    assert len(request(1))==36 and request(1)[:3]==b'AP\1'
    with pytest.raises(ValueError):request(3)
    encoded=b'\0'+struct.pack('<BBBBII8I',1,1,15,0,100,20,1,2,3,4,5,6,7,8)
    assert decode(encoded)['positions_raw']==(1,2,3,4)
    with pytest.raises(ValueError):decode(b'\2')


def test_exported_fk_handles_control_sign_and_home_offset():
    tcp=np.eye(4);tcp[0,3]=1
    fk=MeasuredFK(dict(mount=np.eye(4).tolist(),tcp=tcp.tolist(),
        joints=[dict(origin=np.eye(4).tolist(),axis=[0,0,1])]*5,
        home_model_rad=[np.pi/2,0,0,0,0],control_to_model_sign=[-1]*5))
    assert fk((0,)*12)[:3,3]==pytest.approx((0,1,0))
    assert fk((np.pi/2,)+(0,)*11)[:3,3]==pytest.approx((1,0,0))


def test_actual_pixel_and_joint_producers_feed_session_without_held_injection():
    p=program();color,depth,mask,intr=scene();now=[10.04]
    guard=FixedPlatformGuard(.3,.00002);guard.update(platform(),10.03,10.031)
    inputs=LiveCarryInputs(p,lambda q:np.eye(4),(color,depth),mask,intr,np.eye(4).tolist(),guard,clock=lambda:now[0])
    inputs.add_joints(packet(),10.03,10.034)
    record={'color_time_s':10.025,'depth_time_s':10.026}
    images=(color,(depth*1000).astype(np.uint16),{'depth_scale_m':.001})
    inputs.add_frame(record,images)
    f=inputs.feedback()
    assert f.capture_id==p.capture_id and f.vision_target_id==p.candidate_id
    assert f.payload_relation=='unknown'
    assert f.capture_observed_s==0. and f.scene_validated_s==10.025
    inputs.set_phase('probe')
    now[0]=10.075;inputs.add_joints(packet(completed_pairs=11),10.065,10.069)
    inputs.add_frame({'color_time_s':10.060,'depth_time_s':10.061},images)
    assert inputs.feedback().payload_relation=='contact'  # Static object is not a successful lift.
    now[0]=11.
    with pytest.raises(ValueError,match='stale'):inputs.feedback()


class StopTransport:
    def __init__(self):self.calls=0;self.torque=0;self.position=2000;self.fail=False
    def safe_stop(self):self.calls+=1
    def get_state(self):return NS(stop_latched=True)
    def read_device_inventory(self,bus,motor):
        if self.fail:raise TimeoutError()
        return dict(complete=True,command=dict(torque_enabled_raw=self.torque,goal_speed_raw=0),
                    feedback=dict(position_raw=self.position))


def test_stop_ack_is_not_stop_proof_and_torque_on_never_passes():
    t=StopTransport();now=[10.];s=ResidentStop(t,lambda:now[0])
    s.request('a',True,10);assert s.status=='STOPPING'
    assert s.poll('a',10).status=='STOPPING'
    now[0]=10.3;t.torque=1;assert s.poll('a',10.3).status=='STOPPING'
    now[0]=10.6;t.torque=0;assert s.poll('a',10.6).status=='STOPPING'
    now[0]=10.9;assert s.poll('a',10.9).status=='STOPPED'
    assert t.calls==1


def test_stop_timeout_stays_unconfirmed():
    t=StopTransport();t.fail=True;s=ResidentStop(t)
    s.request('a',True,0)
    assert s.poll('a',1).status=='STOPPING'


@pytest.mark.parametrize('object_follows',[False,True])
def test_follow_evidence_is_derived_from_pixels_and_encoder_fk(object_follows):
    p=program();color,depth,mask,intr=scene();now=[10.04]
    guard=FixedPlatformGuard(.3,.00002);guard.update(platform(),10.03,10.031)
    def fk(q):
        pose=np.eye(4);pose[2,3]=q[2]*.1;return pose
    inputs=LiveCarryInputs(p,fk,(color,depth),mask,intr,np.eye(4).tolist(),guard,clock=lambda:now[0])
    inputs.add_joints(packet(),10.03,10.034)
    images=(color,(depth*1000).astype(np.uint16),{'depth_scale_m':.001})
    inputs.add_frame({'color_time_s':10.025,'depth_time_s':10.026},images);inputs.set_phase('probe')
    now[0]=10.075
    q=(0,0,40000)+(0,)*9
    inputs.add_joints(packet(completed_pairs=11,positions_urad=q),10.065,10.069)
    current=depth+.004 if object_follows else depth
    # Only the tracked object depth changes; static background stays unchanged.
    current=np.where(mask,current,depth)
    inputs.add_frame({'color_time_s':10.060,'depth_time_s':10.061},
                     (color,(current*1000).astype(np.uint16),{'depth_scale_m':.001}))
    assert inputs.feedback().payload_relation==('held' if object_follows else 'unknown')


def test_cached_joint_packet_cannot_renew_measurement_time():
    p=program();color,depth,mask,intr=scene()
    inputs=LiveCarryInputs(p,lambda q:np.eye(4),(color,depth),mask,intr,np.eye(4).tolist(),FixedPlatformGuard(.3,.00002))
    inputs.add_joints(packet(),10.,10.004)
    original=inputs.joints[-1]
    inputs.add_joints(packet(sample_age_ms=(40,35,30,25,20,15)*2),10.01,10.014)
    assert len(inputs.joints)==1 and inputs.joints[-1]==original
    with pytest.raises(ValueError,match='changed positions'):
        inputs.add_joints(packet(positions_urad=(1,)*12),10.01,10.014)
