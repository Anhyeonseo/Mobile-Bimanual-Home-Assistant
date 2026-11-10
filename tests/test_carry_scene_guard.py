from types import SimpleNamespace as NS
import numpy as np
import pytest

from home_robot_tasks.carry_scene_guard import arm_radius, background_depth_guard, route_background_depth_guard


def model(tmp_path):
    record=np.zeros(1,dtype=[('normal','<3f4'),('vertices','<9f4'),('attribute','<u2')])
    record['vertices']=[.1,0,0,.09,.01,0,.09,0,.01]
    (tmp_path/'part.stl').write_bytes(bytes(80)+(1).to_bytes(4,'little')+record.tobytes())
    urdf=tmp_path/'arm.urdf'
    urdf.write_text('''<robot name="test"><link name="root"/><link name="child">
    <collision><origin xyz="0.03 0 0" rpy="0 0 0"/><geometry><mesh filename="part.stl"/></geometry></collision>
    </link><joint name="j" type="revolute"><parent link="root"/><child link="child"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/></joint></robot>''')
    return urdf


def test_collision_mesh_and_joint_translation_bound_every_rotation(tmp_path):
    urdf=model(tmp_path)
    radius=arm_radius(urdf,tmp_path)
    assert radius==pytest.approx(.33)
    for angle in np.linspace(-np.pi,np.pi,101):
        point=np.array([.13*np.cos(angle),.2+.13*np.sin(angle),0])
        assert np.linalg.norm(point)<=radius+1e-8


def test_both_mounts_and_payload_determine_depth_scope(tmp_path):
    urdf=model(tmp_path)
    def arm(z):
        mount=np.eye(4);mount[2,3]=z
        joint=np.eye(4);joint[0,3]=.2
        tcp=np.eye(4);tcp[0,3]=.1
        return NS(mount=mount,joints=[(joint,np.array([0,0,1]))],tcp=tcp)
    arms={'left':arm(.1),'right':arm(.6)}
    grasp={'object_size_m':[.2,.2,.2],'tcp_from_object':np.eye(4).tolist()}
    guard=background_depth_guard(arms,{'base_from_optical':np.eye(4)},grasp,urdf,tmp_path)
    assert guard['maximum_background_depth_m']==pytest.approx(.6+.3+np.sqrt(.03)+.05)
    grasp['object_size_m']=[.4,.4,.4]
    larger=background_depth_guard(arms,{'base_from_optical':np.eye(4)},grasp,urdf,tmp_path)
    assert larger['maximum_background_depth_m']>guard['maximum_background_depth_m']


def test_local_radius_retains_collision_offsets_and_unanchored_branches(tmp_path):
    urdf=model(tmp_path)
    assert arm_radius(urdf,tmp_path,joint_roots={'root','child'})==pytest.approx(.13)
    assert arm_radius(urdf,tmp_path,joint_roots={'root'})==pytest.approx(.33)


def test_route_scope_includes_parked_arm_payload_and_between_sample_excursion(tmp_path):
    from dataclasses import replace
    from home_robot_tasks.s2_geometry import JOINTS,UPSTREAM
    from test_carry_execution import program
    from home_robot_tasks.carry_program import CarryStage
    from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
    urdf=model(tmp_path)
    text=urdf.read_text().replace('name="j"','name="shoulder_pan"')
    extra=''.join(f'<link name="extra{i}"/><joint name="{name}" type="fixed"><parent link="child"/><child link="extra{i}"/><origin xyz="0 0 0"/></joint>' for i,name in enumerate(UPSTREAM[1:5]))
    urdf.write_text(text.replace('</robot>',extra+'</robot>'))
    def arm(z):
        mount=np.eye(4);mount[2,3]=z
        joint=np.eye(4);joint[0,3]=.2
        def fk(q):
            pose=mount.copy();pose[:3,3]+=[.2*np.cos(q[0]),0,.2*np.sin(q[0])]
            return pose,np.array([mount[:3,3],pose[:3,3]])
        return NS(mount=mount,joints=[(joint,np.array([0,1,0]))],tcp=np.eye(4),fk=fk)
    arms={'left':arm(.1),'right':arm(.6)}
    profile={'base_from_optical':np.eye(4).tolist(),'arms':{s:{'alignment':{n:{'home_model_rad':0.,'control_to_model_sign':1} for n in JOINTS}} for s in arms}}
    a=[0.]*12;b=a.copy();a[0]=-.8;b[0]=.8
    stage=CarryStage('fixture',(TimedJointPoint(50,tuple(a)),TimedJointPoint(100,tuple(b))),{},False)
    p=replace(program(),stages=(stage,))
    grasp={'object_size_m':[.02,.02,.02],'tcp_from_object':np.eye(4).tolist()}
    guard=route_background_depth_guard(arms,profile,grasp,urdf,tmp_path,p)
    assert guard['per_arm_interpolation_margin_m']['left']>0
    assert guard['per_arm_interpolation_margin_m']['right']==0
    assert guard['maximum_background_depth_m']>.6+.13
    for q0 in np.linspace(-.8,.8,41):
        for side in arms:
            pose,centers=arms[side].fk([q0 if side=='left' else 0.])
            assert centers[:,2].max()+.13<guard['maximum_background_depth_m']
    big=dict(grasp,object_size_m=[1.,1.,1.])
    assert route_background_depth_guard(arms,profile,big,urdf,tmp_path,p)['maximum_background_depth_m']>guard['maximum_background_depth_m']
    assert guard['checked_samples']==2 and guard['collision_scene_changed'] is False
