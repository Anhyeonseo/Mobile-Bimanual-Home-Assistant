from copy import deepcopy
import pytest
from test_runtime_stack import read, request
from home_robot_tasks.offline_stack import OfflineStack


def stack(guarded=False):
    return OfflineStack(read('fetch_stack.simulation.json'),read('home.example.json'),
        read('navigation_map.simulation.json'),joint_placement=True,guarded_navigation=guarded)


@pytest.mark.parametrize('guarded',[False,True])
def test_joint_placement_runs_all_fetch_stages_with_fresh_post_arrival_captures(guarded):
    s=stack(guarded);result=s.run(request())
    assert result['status']=='SUCCEEDED' and result['completed_steps']==15
    assert result['hardware_commands']==0 and not result['physical_task_completed']
    assert [h['operation'] for h in result['placement_history']]==['pick','place']
    for item in result['placement_history']:
        selected=s.assembly.catalog.placement_policy.selected[(item['operation'],'remote_control')]
        assert selected['capture_id']!=item['final_capture_id']
        assert abs(item['current_height_m']-item['candidate']['lift_height_m'])<.003
    assert not s.held and s.attached is None


@pytest.mark.parametrize('fault',['sensor_loss','load_loss','reboot'])
def test_joint_placement_does_not_continue_after_payload_or_state_loss(fault):
    s=stack();result=s.run(request(),fault=fault)
    assert result['status']=='STOP_UNCONFIRMED' and result['stop_requests']==1
    assert [h['operation'] for h in result['placement_history']]==['pick']
    assert not any(h['phase'].startswith('place_') for h in result['manipulation_stages'])


@pytest.mark.parametrize('step',range(15))
def test_joint_placement_can_cancel_at_each_fetch_stage(step):
    s=stack();result=s.run(request(),cancel_step=step)
    assert result['status']=='CANCELLED' and result['stop_requests']==1


def test_changed_scene_during_alignment_blocks_arm_dispatch():
    s=stack();s.runtime.submit(request());task=s.runtime.app.tasks['stack']
    changed=False
    for _ in range(1000):
        if task.step['skill']=='adjust_lift_for_pick' and not changed:
            s.scene_revision='changed-scene';changed=True
        s.tick()
        if task.state in task.TERMINAL or task.state=='STOP_UNCONFIRMED':break
    assert changed and task.state!='SUCCEEDED'
    assert not s.ports['gripper'].starts


@pytest.mark.parametrize('source,destination', [((2.,2.),(6.,4.)),((2.,3.),(7.,4.)),((3.,4.),(8.,2.))])
def test_joint_placement_is_not_tied_to_one_registered_approach(source,destination):
    config=read('fetch_stack.simulation.json')
    for name,xy in [('sofa',source),('bedroom_drop_zone',destination)]:
        place=config['catalog']['places'][name]
        for approach in [place['approach']]+[v['approach'] for v in place['views']]:
            approach['point'].update(x=xy[0],y=xy[1])
    s=OfflineStack(config,read('home.example.json'),read('navigation_map.simulation.json'),joint_placement=True)
    result=s.run(request())
    assert result['status']=='SUCCEEDED'
    assert len(result['placement_history'])==2
