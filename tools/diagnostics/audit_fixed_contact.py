#!/usr/bin/env python3
"""Review signed SO101 contact geometry from an existing report. No device I/O."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.run_s3_stationary_lab import model
from home_robot_tasks.carry_planning import GraspRouteCandidate,fixed_contact_review
from home_robot_tasks.carry_binding import bind_observed_fixed_contact
from home_robot_tasks.carry_contact_ik import fit_pregrasp
from home_robot_tasks.carry_cartesian import bounded_cartesian
from home_robot_tasks.grasp_preferences import TabletopGraspPolicy
from home_robot_tasks import s2_geometry as geo

def review(report, observed_state=None):
    lift=report['initial_model_joint_positions_si']['s2_lift_joint'];arm=model(lift)
    raw=report['plan']['resolved_grasp'];candidate=GraspRouteCandidate(**raw)
    tcp=np.asarray(candidate.base_from_grasp_tcp);obj=tcp@np.asarray(candidate.tcp_from_object)
    vertices=np.asarray(candidate.payload_mesh['vertices'])@obj[:3,:3].T+obj[:3,3]
    original=fixed_contact_review(arm,candidate)
    grip,section,correction=bind_observed_fixed_contact(arm,tcp,vertices)
    width=section['width_m'];opened=width+.004
    corrected=replace(candidate,base_from_grasp_tcp=grip,tcp_from_object=np.linalg.inv(grip)@obj,
        open_jaw_model_rad=float(np.interp(opened,arm.jaw.gaps,arm.jaw.angles)),
        held_jaw_model_rad=float(np.interp(width,arm.jaw.gaps,arm.jaw.angles)))
    intended=fixed_contact_review(arm,corrected)
    ok,pre,goal,ik=fit_pregrasp(arm,grip,np.array(report['contact_model_rad']),.02,
        contact_roll=True,grasp_policy=TabletopGraspPolicy(),approach_mode='vertical')
    result=dict(status='OFFLINE_CONTACT_REVIEW',original=original,corrected=intended,correction=correction,
        pregrasp_ik_accepted=bool(ok),pregrasp_ik=ik,base_delta_m=[0.,0.,0.],lift_travel_m=lift,
        full_stl_route_checked=False,motion_authorized=False,execution_ready=False)
    if ok:
        try:
            path=bounded_cartesian(arm,pre,goal);actual=arm.fk(path[-1])[0]
            fitted=replace(corrected,pregrasp_model_rad=pre,base_from_grasp_tcp=actual,tcp_from_object=np.linalg.inv(actual)@obj)
            result.update(fitted=fixed_contact_review(arm,fitted),contact_model_rad=path[-1].tolist(),
                candidate={**raw,'pregrasp_model_rad':pre.tolist(),'base_from_grasp_tcp':actual.tolist(),
                    'tcp_from_object':fitted.tcp_from_object.tolist(),
                    'open_jaw_model_rad':fitted.open_jaw_model_rad,'held_jaw_model_rad':fitted.held_jaw_model_rad})
        except ValueError as exc:result['cartesian_error']=str(exc)
    _,axes=np.linalg.eigh(np.cov(vertices[:,:2].T));axis=axes[:,-1]
    lengths=vertices[:,:2]@axis;lo=lengths.min();span=np.ptp(lengths)
    old_center=np.asarray(report['contact_base_m'])
    result['longitudinal_review']=dict(axis_xy=axis.tolist(),span_m=float(span),
        planned_fraction_from_model_end=float((old_center[:2]@axis-lo)/span),
        user_reported_fraction_from_visual_top=.75,model_end_is_not_identified_visual_top=True,
        actual_contact_verified=False,mass_center_measured=False)
    if observed_state is not None:
        q=observed_state['last_feedback']['positions_rad']
        qm=[geo.model_from_control(q[i],arm.profile['arms']['left']['alignment'][n]) for i,n in enumerate(geo.JOINTS[:5])]
        actual=arm.fk(qm)[0]@arm.tcp_from_stock
        center=actual[:3,3]-actual[:3,0]*arm.jaw.center_from_fixed_m(original['held_gap_m'])
        result['longitudinal_review'].update(encoder_fk_fraction_from_model_end=float((center[:2]@axis-lo)/span),
            encoder_fk_center_base_m=center.tolist(),sample_phase='near probe endpoint, not independent physical contact')
    return result,vertices,axis,lo,span

def draw(result,vertices,axis,lo,span,destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cross=np.array([-axis[1],axis[0]]);mid=vertices[:,:2].mean(0)
    def project(points):
        p=np.atleast_2d(points)[:,:2]
        return np.c_[(p-mid)@cross*1000,(p@axis-lo)/span*100]
    shape=project(vertices);h=ConvexHull(shape)
    fig,axs=plt.subplots(1,2,figsize=(11,7),layout='constrained')
    for ax,key,title in zip(axs,['original','corrected'],['Executed target: fixed finger inside','Corrected geometry target (not executed)']):
        r=result[key];ax.fill(*shape[h.vertices].T,color='#dedede',label='Observed object envelope')
        pairs=project([r['fixed_contact_base_m'],r['moving_contact_base_m']]);face=project(r['fixed_face_base_m'])[0]
        moving=np.array(r['fixed_face_base_m'])+np.array(r['fixed_to_moving_base'])*r['open_gap_m'];opened=project(moving)[0]
        ax.plot(*pairs.T,'--',color='#667085',label='Actual section through object')
        ax.plot([face[0],opened[0]],[face[1],opened[1]],color='#6296ad',lw=2,label='Open gap')
        ax.scatter(*face,s=110,color='#cf3b32',label='Fixed contact face',zorder=5)
        ax.scatter(*opened,s=90,color='#287ca5',marker='s',label='Moving face while open',zorder=5)
        ax.axhline(50,color='#777',lw=1,ls=':');ax.text(-43,47,'Shape midpoint',fontsize=9)
        ax.set(title=title,xlabel='Across object (mm)',ylabel='Length from model end (%)',ylim=(-8,108),xlim=(-48,48))
        ax.grid(alpha=.15);ax.legend(loc='upper right',fontsize=8)
    fig.suptitle('SO101 fixed-side binding: observed geometry, not a live photograph',fontsize=14)
    fig.savefig(destination,dpi=160);plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,required=True);parser.add_argument('--observed-state',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    r,v,a,lo,span=review(json.loads(args.report.read_text()),json.loads(args.observed_state.read_text()) if args.observed_state else None)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'review.json').write_text(json.dumps(r,indent=2,allow_nan=False))
    draw(r,v,a,lo,span,args.output/'fixed-contact-comparison.png')
    print(json.dumps({k:r[k] for k in ['status','pregrasp_ik_accepted','longitudinal_review']}))
    print('fitted',r.get('fitted',r.get('cartesian_error')))
