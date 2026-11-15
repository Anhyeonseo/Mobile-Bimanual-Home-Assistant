import copy
import math

import numpy as np
import pytest

from home_robot_tasks.assistant.world import WorldStore
from home_robot_tasks.furniture_relocation import fit_relocation,FurnitureLocationMemory


def match(dx=.4):
    rng=np.random.default_rng(41)
    source=np.c_[rng.uniform(-.4,.4,40),rng.uniform(-.25,.25,40),np.full(40,.35)]
    a=.3;rotation=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    target=source.copy();target[:,:2]=source[:,:2]@rotation.T+[dx,-.15]
    target+=rng.normal(0,.002,target.shape)
    proposal=fit_relocation(source,target)
    assert proposal is not None
    return source,target,proposal


def test_rigid_rgbd_motion_rejects_outliers_but_not_real_translation():
    source,target,p=match()
    assert np.asarray(p['matrix'])[:2,3]==pytest.approx([.4,-.15],abs=.01)
    target[-6:,:2]+=[1.,1.]
    p=fit_relocation(source,target)
    assert p is not None and p['inliers']>=32


def test_collapsed_matches_scale_change_and_wrong_height_are_not_identity():
    source,target,_=match()
    assert fit_relocation(source,np.tile(target[0],(len(target),1))) is None
    wrong=target.copy();wrong[:,:2]*=1.5
    assert fit_relocation(source,wrong) is None
    wrong=target.copy();wrong[:,2]+=.2
    assert fit_relocation(source,wrong) is None
    assert fit_relocation(source[:7],target[:7]) is None


def inputs():
    _,_,proposal=match()
    reference=dict(furniture_id='table-1',revision='reference-1',anchor_xyz_m=[0.,0.,.35])
    proposal.update(furniture_id='table-1',reference_revision='reference-1',anchor_xyz_m=(np.asarray(proposal['matrix'])[:3,:3]@reference['anchor_xyz_m']+np.asarray(proposal['matrix'])[:3,3]).tolist())
    return reference,proposal


def test_last_verified_location_survives_absence_and_process_restart(tmp_path):
    ref,p=inputs();path=tmp_path/'world.sqlite';store=WorldStore(path)
    memory=FurnitureLocationMemory(ref,store,'boot-1')
    for t in (1.,2.):assert memory.observe(p,capture_id=str(t),observed_s=t,now_s=t,localization_qualified=True) is None
    result=memory.observe(p,capture_id='3',observed_s=3.,now_s=3.,localization_qualified=True)
    assert result['anchor_xyz_m']==p['anchor_xyz_m']
    assert result['occupancy_clearance_authorized'] is False
    memory.observe(None,capture_id='4',observed_s=4.,now_s=4.,localization_qualified=True)
    assert memory.saved==result
    store.close();store=WorldStore(path);again=FurnitureLocationMemory(ref,store,'boot-2')
    assert again.saved==result and not again.votes
    store.close()


def test_stale_unlocalized_disagreeing_or_replayed_observation_cannot_replace_memory():
    ref,p=inputs();store=WorldStore();memory=FurnitureLocationMemory(ref,store,'clock')
    for values in (dict(observed_s=1.,now_s=7.,localization_qualified=True),dict(observed_s=1.,now_s=1.,localization_qualified=False)):
        with pytest.raises(ValueError):memory.observe(p,capture_id='bad',**values)
    memory.observe(p,capture_id='one',observed_s=1.,now_s=1.,localization_qualified=True)
    with pytest.raises(ValueError):memory.observe(p,capture_id='one',observed_s=1.,now_s=1.,localization_qualified=True)
    q=copy.deepcopy(p);q['anchor_xyz_m'][0]+=.5;q['matrix'][0][3]+=.5
    assert memory.observe(q,capture_id='two',observed_s=2.,now_s=2.,localization_qualified=True) is None
    assert len(memory.votes)==1 and memory.saved is None
    store.close()


def test_anchor_cannot_disagree_with_geometric_transform():
    ref,p=inputs();p['anchor_xyz_m'][0]+=.1
    store=WorldStore();memory=FurnitureLocationMemory(ref,store,'clock')
    with pytest.raises(ValueError,match='anchor'):
        memory.observe(p,capture_id='bad',observed_s=1.,now_s=1.,localization_qualified=True)
    assert memory.saved is None
    store.close()
