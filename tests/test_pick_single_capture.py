from types import SimpleNamespace as NS
import pytest
from home_robot_tasks.pick_web_backend import WebPickBackend
from home_robot_tasks.pick_backend import PipelineBackend
from tools.diagnostics.prepare_carry_execution import review_top_clearance
from test_carry_snapshot_execution import admitted_inputs

def test_top_completion_never_recaptures_or_claims_physical_success():
 b=WebPickBackend(dict(mode='station'));b.hardware=NS(session=NS(state='HOLDING'))
 b._compute=lambda *a,**k:pytest.fail('no second image processing')
 out=b.verify(NS(note=lambda *a:None),dict(operation='top-pick',bundle='checked'))
 assert out['status']=='UNVERIFIED' and out['physical_success'] is False
 assert out['verification_basis']=='operator_observation' and out['verification_samples']==[]

def test_other_modes_keep_their_existing_verification(monkeypatch):
 b=WebPickBackend(dict(mode='station'));expected=object()
 monkeypatch.setattr(PipelineBackend,'verify',lambda *a:expected)
 assert b.verify(None,dict(operation='observe-rise-pick')) is expected

def test_single_capture_skips_target_comparison_without_relabelling_live_evidence():
 inputs,now,record,images=admitted_inputs();inputs.fixed_planning_snapshot=True;inputs.validation=None
 inputs.scene.compare=lambda *a,**k:pytest.fail('no second image comparison')
 inputs.add_frame(record,images);inputs.admit_snapshot()
 assert inputs.validation['live_target_revalidated'] is False
 assert inputs.validation['fixed_planning_snapshot'] is True
 assert inputs.validation['fixed_bottom_observation'] is False
 assert inputs.feedback().payload_relation=='unknown'

def test_historical_four_mm_does_not_reject_positive_model_clearance():
 r=review_top_clearance(dict(stages=[dict(stage='close',minimum_by_finger={
    'fixed':dict(clearance_m=.0030589),'moving':dict(clearance_m=.0019537)})]))
 assert r['hard_limit'] is False and len(r['below_historical_residual'])==2
 assert r['minimum_clearance_m']==.0019537 and r['physical_accuracy_verified'] is False
