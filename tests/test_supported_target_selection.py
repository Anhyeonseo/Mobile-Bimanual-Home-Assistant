"""Recorded false tabletop proposal must not pin selection to an old position."""
from copy import deepcopy
import json
from pathlib import Path
import pytest
from tools.diagnostics.review_s3_capture import choose_observation

@pytest.fixture
def case():
 r=json.loads((Path(__file__).parent/'fixtures/target_ambiguity_20261120.json').read_text())
 return r['observations'],r['meta'],r['profile'],r['source']

def choose(case,location=None):
 obs,meta,profile,source=case
 return choose_observation(obs,obs[0]['capture_id'],profile,meta,source,'white eraser',location)

@pytest.mark.parametrize('reverse,box',[(False,[301,260,351,345]),(True,[30,300,80,380]),(False,[500,100,550,180])])
def test_supported_target_is_selected_at_new_locations_without_dropping_other_observation(case,reverse,box):
 obs,*_=case;obs[0]['box_xyxy']=box
 if reverse:obs.reverse()
 before=deepcopy(obs);result=choose(case)
 assert result['status']=='SELECTED'
 assert obs[result['observation_index']]['box_xyxy']==box
 assert result['target_selection']['method']=='dominant_detection'
 assert obs==before

@pytest.mark.parametrize('change',['missing_score','close_score','invalid_score'])
def test_real_or_unresolved_alternatives_remain_ambiguous(case,change):
 obs,*_=case
 if change=='missing_score':obs[1].pop('detection_score')
 if change=='close_score':obs[1]['detection_score']=.75
 if change=='invalid_score':obs[0]['detection_score']=float('nan')
 assert choose(case)['status']=='AMBIGUOUS_TARGET'

def test_explicit_left_does_not_substitute_the_supported_right_object(case):
 assert choose(case,'left')['status']=='REOBSERVE'
 assert choose(case,'right')['status']=='SELECTED'


@pytest.mark.parametrize('status,reason,failure',[
 ('held','no_locally_verified_support',None),
 ('rejected','no_coherent_cluster','INVALID_DEPTH'),
 ('held','partial_mask_depth','INVALID_DEPTH'),
 ('accepted',None,None),
])
def test_weak_alternative_geometry_cannot_flip_target_identity(case,status,reason,failure):
 obs,*_=case;obs[1].update(status=status,reason=reason,failure=failure)
 result=choose(case)
 assert result['status']=='SELECTED' and result['observation_index']==0


def test_bad_depth_on_dominant_target_does_not_substitute_weaker_object(case):
 obs,*_=case
 obs[0].update(status='rejected',reason='no_coherent_cluster',failure='INVALID_DEPTH')
 obs[1].update(status='accepted',reason=None,failure=None)
 result=choose(case)
 assert result['status']=='REOBSERVE'
 assert result['target_selection']['selected_index']==0


def test_two_equally_likely_supported_objects_are_still_ambiguous(case):
 obs,*_=case;obs[1].update(status='accepted',reason=None,failure=None,detection_score=.79)
 assert choose(case)['status']=='AMBIGUOUS_TARGET'


def test_second_recorded_failure_selects_the_same_real_eraser():
 r=json.loads((Path(__file__).parent/'fixtures/target_ambiguity_repeat_20261120.json').read_text())
 result=choose((r['observations'],r['meta'],r['profile'],r['source']))
 assert result['status']=='SELECTED' and result['observation_index']==0
