import numpy as np
import pytest
from home_robot_tasks.carry_planning import plan_pick_carry,PlanRejected
from tools.diagnostics.plan_capture_carry import tabletop_lift_directions
from test_carry_planning import arm,candidate,FakePlanner


def test_measured_table_normal_precedes_vertical_without_expanding_probe_envelope():
    normal=[-.0203275292047642,-.002196496405218504,.9997909616314661]
    directions=tabletop_lift_directions(normal)
    np.testing.assert_allclose(directions[0],normal)
    assert directions[1]==[0.,0.,1.]
    assert np.linalg.norm(np.asarray(directions[0])[:2])*.02<.002
    assert tabletop_lift_directions([0,0,2])==[[0.,0.,1.]]
    assert tabletop_lift_directions([.3,0,1])==[[0.,0.,1.]]


@pytest.mark.parametrize('normal',[[0,0,0],[0,0,-1],[np.nan,0,1],[0,1]])
def test_invalid_plane_cannot_authorize_a_lift(normal):
    with pytest.raises(ValueError):tabletop_lift_directions(normal)


@pytest.mark.parametrize('blocked',[0,1,2])
def test_each_lift_alternative_keeps_collision_check_and_failures(arm,blocked):
    class Planner(FakePlanner):
        def __init__(self):super().__init__();self.goals=[]
        def cartesian(self,start,goal,c,*,attached,label):
            if label=='lift':
                assert attached
                self.goals.append(goal[:3,3]-c.base_from_grasp_tcp[:3,3])
                if len(self.goals)<=blocked:raise PlanRejected('real obstacle')
            return super().cartesian(start,goal,c,attached=attached,label=label)
    planner=Planner();directions=tabletop_lift_directions([-.02,0,1])
    result=plan_pick_carry(arm,planner,arm.anchor,[candidate(arm)],scene_id='scene-1',
                          lift_directions_base=directions)
    assert len(planner.goals)==min(blocked+1,2)
    for delta,direction in zip(planner.goals,directions):np.testing.assert_allclose(delta,np.asarray(direction)*.02,atol=1e-12)
    if blocked==2:
        assert result.status=='NO_COMPLETE_ROUTE'
        attempts=result.failures[0]['diagnostics']['lift_attempts']
        assert len(attempts)==2 and all(a['reason']=='real obstacle' for a in attempts)
    else:
        assert result.status=='OFFLINE_ROUTE_FOUND'
        assert [p.label for p in result.stages][-3:]==['close_geometry','lift','carry']
