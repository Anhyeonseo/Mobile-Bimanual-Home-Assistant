from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from home_robot_tasks import s2_geometry as geo
from home_robot_tasks.stationary_pick_sim import StationaryArm
from home_robot_tasks.carry_posture import checked_joints
from so101_arm_bridge.servo_position import single_turn_window
C=Path(__file__).resolve().parents[1]/'config/home_assistant'
def inputs():
 return (json.loads((C/'s2_coordinates.development.json').read_text()),geo.load_geometry(C/'official_geometry.json'),json.loads((C/'arm_ranges.development.json').read_text()))
def test_actual_captured_shoulder_is_valid_without_changing_measured_angle():
 p,g,r=inputs();a=StationaryArm(p,g,r,.63,None)
 q=np.array([-.026078,-1.7669101500207056,1.327477339180908,1.2051172838818083,1.607612])
 assert np.degrees(q[1]) < -100
 np.testing.assert_array_equal(checked_joints(a,q),q)
 for i,n in enumerate(geo.JOINTS[:5]):
  expected=sorted(geo.model_angle_from_raw(x,p['arms']['left']['alignment'][n],n) for x in single_turn_window(r['arms']['left'][n]))
  np.testing.assert_allclose([a.lo[i],a.hi[i]],expected)
 q[1]=a.lo[1]-.001
 with pytest.raises(ValueError,match='shoulder=.*allowed'):checked_joints(a,q)
def test_unmeasured_ranges_do_not_expand_cad_limits():
 p,g,r=inputs()
 for v in r['arms']['left'].values():v['measured_on_this_arm']=False
 a=StationaryArm(p,g,r,.63,None)
 assert np.degrees(a.lo[1]) == pytest.approx(-100,abs=.001)
 with pytest.raises(ValueError,match='shoulder'):checked_joints(a,[-.026078,-1.76691,1.32748,1.20512,1.60761])
@pytest.mark.parametrize('q',[[0]*4,[0,0,0,0,float('nan')]])
def test_invalid_coordinates_still_fail(q):
 a=StationaryArm(*inputs(),.63,None)
 with pytest.raises(ValueError,match='five finite'):checked_joints(a,q)
