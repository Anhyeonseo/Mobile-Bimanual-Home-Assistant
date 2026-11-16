#!/usr/bin/env python3
"""Render an archived directional candidate, physical stock fingers and approach."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/diagnostics'),str(ROOT/'ros2_ws/src/home_robot_tasks'),str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from tools.diagnostics.run_s3_stationary_lab import model
from home_robot_tasks.assistant.object_alignment import contact_direction_review,reach_comfort_review

def render_arm_pose(trial, review):
    """Reuse the pinned viewer's mesh bundle at the exact candidate joint angles."""
    import base64
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from home_robot_tasks import s2_geometry as geo
    result=json.loads((review/'candidates.json').read_text());c=result['candidates'][0]
    arm=model(c['lift_travel_m']);q=np.array(c['fk_witness_model_rad'])
    geometry=geo.load_geometry(ROOT/'config/home_assistant/official_geometry.json')
    surfaces=json.loads((ROOT/'tools/diagnostics/s2_viewer/meshes.json').read_text())
    if surfaces['geometry_sha256']!=geo.digest(geometry):raise ValueError('display geometry revision mismatch')
    angles=dict(zip(geo.UPSTREAM[:5],q));angles['gripper']=float(np.interp(c['entry_review']['contact_width_m'],arm.jaw.gaps,arm.jaw.angles))
    frames={'base_link':arm.mount.copy()};pending=list(geometry['arm_joints'])
    while pending:
        progress=False
        for j in pending[:]:
            if j['parent']['link'] not in frames:continue
            pose=frames[j['parent']['link']]@geo.origin(j)
            if j['type']!='fixed':
                axis=np.fromstring(j['axis']['xyz'],sep=' ')
                pose[:3,:3]=pose[:3,:3]@Rotation.from_rotvec(axis*angles[j['name']]).as_matrix()
            frames[j['child']['link']]=pose;pending.remove(j);progress=True
        if not progress:raise ValueError('disconnected official arm tree')
    np.testing.assert_allclose(frames['gripper_frame_link'],arm.fk(q)[0]@arm.tcp_from_stock,atol=1e-10)
    x,y,yaw=c['base_xy_yaw'];co,si=np.cos(yaw),np.sin(yaw)
    world=np.array([[co,-si,0,x],[si,co,0,y],[0,0,1,-arm.profile['ground_z_m']],[0,0,0,1.]])
    center=np.array(c['contact_direction']['contact_center_map_m']);axis=np.array(c['contact_direction']['shape_axis_xy']);lateral=np.array([-axis[1],axis[0]])
    basis=np.array([[*axis,0],[*lateral,0],[0,0,1.]])
    def local(vertices):
        vertices=np.array(vertices);out=vertices.copy();out[:,:2]-=center[:2];return out@basis.T
    def apply(vertices,T):return np.asarray(vertices)@T[:3,:3].T+T[:3,3]
    meshes=[]
    for visual in surfaces['arm_visuals']:
        m=surfaces['meshes'][visual['mesh']]
        verts=np.frombuffer(base64.b64decode(m['vertices']),dtype='<u2').reshape(-1,3)/65535*np.array(m['span'])+m['min']
        faces=np.frombuffer(base64.b64decode(m['faces']),dtype='<u2').reshape(-1,3)
        v=local(apply(verts,world@frames[visual['link']]@np.array(visual['origin'])))
        color='#e7b945' if visual['material']=='arm' else '#3d4650'
        if visual['mesh']=='wrist_roll_follower_so101_v1.stl':color='#e9843b'
        if visual['mesh']=='moving_jaw_so101_v1.stl':color='#2a9ac7'
        meshes.append((v,faces,color))
    points=np.array(json.loads((trial/'height-corrected-alignment/alignment-scene.json').read_text())['points_map_m']);points=local(points)
    chain=local(apply(arm.fk(q)[1],world))
    names=['베이스 회전','어깨','팔꿈치','손목 굽힘','손목 회전']
    limits=np.degrees(np.c_[arm.lo,arm.hi]);deg=np.degrees(q)
    margin=np.degrees(np.minimum(q-arm.lo,arm.hi-q))
    # The bend between actual link-centre segments differs from the URDF joint angle.
    def included(i):
        a=chain[i-1]-chain[i];b=chain[i+1]-chain[i]
        return float(np.degrees(np.arccos(np.clip(a@b/np.linalg.norm(a)/np.linalg.norm(b),-1,1))))
    allv=np.vstack([v for v,_,_ in meshes]);lo=allv.min(0);hi=allv.max(0)
    fig=plt.figure(figsize=(16,8));gs=fig.add_gridspec(2,3,height_ratios=[3.0,1],top=.78,bottom=.09,wspace=.03,hspace=.05)
    views=[(25,-65,'① 입체 모습'),(0,-90,'② 옆면 · 장축 방향으로 접근'),(90,-90,'③ 위에서 본 모습')]
    for index,(elev,azim,title) in enumerate(views):
        ax=fig.add_subplot(gs[0,index],projection='3d');ax.set_proj_type('ortho')
        for v,faces,color in meshes:
            ax.add_collection3d(Poly3DCollection(v[faces],facecolors=color,linewidths=0,alpha=1,shade=True,lightsource=matplotlib.colors.LightSource(azdeg=220,altdeg=45)))
        support=result['support_z_m'];table=np.array([[-.10,-.13,support],[.15,-.13,support],[.15,.13,support],[-.10,.13,support]])
        ax.add_collection3d(Poly3DCollection([table],facecolor='#dcd7cb',alpha=.4,edgecolor='#b5aa95',linewidth=.5))
        ax.scatter(points[::4,0],points[::4,1],points[::4,2],s=2,c='#505966',alpha=.55,depthshade=False)
        for i,name in [(2,'어깨'),(3,'팔꿈치'),(4,'손목')]:
            point=chain[i];ax.scatter(*point,s=22,c='#b93072',depthshade=False)
            if index==1:ax.text(point[0]-.025,point[1]-.05,point[2]+.025,name,color='#a12661',fontsize=10)
        approach=np.array(c['contact_direction']['approach_map'])@basis.T
        pos=np.array([0.,0.,center[2]])-.065*approach
        ax.quiver(*pos,*approach,length=.065,color='#14846a',linewidth=2,arrow_length_ratio=.2)
        ax.set(xlim=(lo[0]-.04,.16),ylim=(min(lo[1]-.03,-.14),max(hi[1]+.03,.14)),zlim=(support-.02,hi[2]+.055))
        ax.set_box_aspect((.16-lo[0]+.04,max(hi[1]+.03,.14)-min(lo[1]-.03,-.14),hi[2]+.075-support),zoom=1.40)
        ax.view_init(elev=elev,azim=azim);fig.text((ax.get_position().x0+ax.get_position().x1)/2,.815,title,ha='center',fontsize=14);ax.set_axis_off()
    ax=fig.add_subplot(gs[1,:]);ax.axis('off')
    rows=[[name,f'{value:+.1f}°',f'{bounds[0]:+.1f}° ~ {bounds[1]:+.1f}°',f'{m:.1f}°'] for name,value,bounds,m in zip(names,deg,limits,margin)]
    t=ax.table(cellText=rows,colLabels=['관절','후보 모델각','현재 모델의 허용 범위','가장 가까운 한계까지'],cellLoc='center',bbox=[.10,0,.8,1.04]);t.auto_set_font_size(False);t.set_fontsize(10.5)
    for (r,col),cell in t.get_celld().items():
        cell.set_edgecolor('#d5dce4');cell.set_facecolor('#e9f2ef' if r==0 else '#ffffff')
    fig.suptitle('추천 후보의 실제 SO-ARM101 링크 형상',fontsize=22,fontweight='bold',y=.975)
    fig.text(.5,.925,'노랑: 팔 구조물  ·  검정: 모터  ·  주황: 고정 손가락  ·  파랑: 이동 손가락  ·  초록: 접근 방향',ha='center',fontsize=12)
    fig.text(.5,.885,f'팔꿈치 링크 사이 각도 약 {included(3):.1f}°  |  리프트 +1cm  |  왼팔만 표시 · 리프트/본체 생략',ha='center',fontsize=12,color='#4d626e')
    fig.text(.5,.035,'공식 URDF + 표시용 STL 메시에 후보 관절각 적용 · 관절 범위 확인됨 · 자기충돌/전체 이동 경로/실물 성공은 아직 검증 전',ha='center',fontsize=11,color='#536271')
    fig.savefig(review/'candidate-arm-pose.png',dpi=150)
    report=dict(candidate_id=c['candidate_id'],joint_model_deg=deg.tolist(),joint_limits_deg=limits.tolist(),nearest_limit_margin_deg=margin.tolist(),elbow_included_angle_deg=included(3),wrist_included_angle_deg=included(4),all_joint_limits_passed=bool(np.all(q>=arm.lo)&np.all(q<=arm.hi)),urdf_fk_matches_planner=True,full_self_collision_checked=False,motion_executed=False)
    (review/'arm-pose-review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True);p.add_argument('--review',type=Path,required=True);a=p.parse_args()
    load=lambda path:json.loads(path.read_text())
    result=load(a.review/'candidates.json');new=result['candidates'][0];st=load(a.trial/'align-16-review/final-status.json')
    scene=load(a.trial/'height-corrected-alignment/alignment-scene.json');points=np.array(scene['points_map_m'])
    old=load(a.trial/'approved-executable-alignment.json')['candidate'];old['observed_region']['long_axis_xy']=new['observed_region']['long_axis_xy']
    arm=model(0);old_d=contact_direction_review(arm,old);old_reach=reach_comfort_review(arm,old)
    direction=new['contact_direction'];center=np.array(direction['contact_center_map_m']);axis=np.array(direction['shape_axis_xy'])
    body=np.array(st['navigation_body']['footprint_xy_m']);start=np.array(st['pose']);goal=np.array(new['base_xy_yaw'])
    def transform(pose):
        x,y,yaw=pose;c,s=np.cos(yaw),np.sin(yaw)
        return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,-arm.profile['ground_z_m']],[0,0,0,1.]])
    def apply(pts,T):return np.asarray(pts)@T[:3,:3].T+T[:3,3]
    def world_arm(candidate):
        tool,chain=arm.fk(candidate['fk_witness_model_rad']);T=transform(candidate['base_xy_yaw']);T[:3,3]+=candidate['lift_travel_m']*np.asarray(arm.profile['lift']['axis_base'])
        return T@tool@arm.tcp_from_stock,apply(chain,T)
    stock,chain=world_arm(new);oldstock,oldchain=world_arm(old)
    width=new['entry_review']['contact_width_m'];opening=new['entry_review']['open_width_m']
    def vertices(width):
        angle=float(np.interp(width,arm.jaw.gaps,arm.jaw.angles));R=Rotation.from_rotvec(arm.jaw.hinge_axis*angle).as_matrix()
        return apply(arm.jaw.fixed_vertices,stock),apply((arm.jaw.moving_vertices-arm.jaw.hinge_point)@R.T+arm.jaw.hinge_point,stock)
    fixed,moving=vertices(width);_,opened=vertices(opening)
    for f in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',):font_manager.fontManager.addfont(f)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':11,'axes.unicode_minus':False})
    fig,axs=plt.subplots(2,2,figsize=(14,11.5),gridspec_kw={'height_ratios':[1.15,1]});fig.subplots_adjust(top=.875,bottom=.10,hspace=.34,wspace=.28)
    fig.suptitle('집기 베이스 수정 후보 · 왼팔 접근 방향을 물체 장축에 맞춤',fontsize=20,fontweight='bold',y=.965)
    fig.text(.5,.925,'저장된 촬영·정지 위치로 재계산한 모델 예측 | 실제 이동·리프트 상승·집기 미실행',ha='center',color='#5c6778')
    current_color='#c17817';oldcolor='#537eaa';newcolor='#14846a'
    ax=axs[0,0]
    f=st['furniture'];table=apply(f['reference_geometry'],np.array(f['last_location']['matrix']))
    hull=table[ConvexHull(table[:,:2]).vertices,:2]
    ax.add_patch(Polygon(hull,facecolor='#ece9e1',edgecolor='#aaa28e',label='이동 반영 탁자 관측 외곽'))
    def footprint(pose,color,label,linestyle='-'):
        T=transform(pose);poly=apply(np.c_[body,np.zeros(len(body))],T)[:,:2]
        ax.add_patch(Polygon(poly,facecolor='none',edgecolor=color,lw=2,linestyle=linestyle,label=label))
        ax.arrow(pose[0],pose[1],.12*np.cos(pose[2]),.12*np.sin(pose[2]),head_width=.025,color=color,length_includes_head=True)
    footprint(old['base_xy_yaw'],oldcolor,'기존 목표 · 베이스/리프트 외형','--');footprint(goal,newcolor,'새 후보 · 베이스/리프트 외형')
    ax.scatter(*start[:2],c=current_color,s=45,label='마지막 정지 위치',zorder=8)
    route=np.array(new['navigation_plan']['poses_xy_yaw']);ax.plot(route[:,0],route[:,1],color=newcolor,lw=2,ls=':')
    ax.plot(oldchain[:,0],oldchain[:,1],color=oldcolor,alpha=.6,lw=2,label='기존 왼팔 계산 자세')
    ax.plot(chain[:,0],chain[:,1],'-o',color=newcolor,lw=2,ms=3,label='새 왼팔 계산 자세')
    ax.scatter(points[:,0],points[:,1],s=2,color='#292c32');ax.set_xlim(-.7,.35);ax.set_ylim(.3,1.75)
    ax.set_title('① 전체 배치 · 화살표는 본체 정면');ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.grid(alpha=.15)
    ax=axs[0,1];ax.scatter(points[:,0],points[:,1],s=4,color='#666e79',alpha=.35,label='리모컨 관측 점')
    for v,col,label in ((fixed,'#d26c27','고정 손가락 STL'),(moving,'#168bba','이동 손가락 STL')):
        ax.scatter(v[::3,0],v[::3,1],s=.5,color=col,alpha=.28,label=label)
    ax.plot(*(np.array([center[:2]-.15*axis,center[:2]+.15*axis]).T),color='#32363d',ls='--',lw=1,label='물체 장축')
    for d,col,label in ((old_d,oldcolor,'기존 접근'),(direction,newcolor,'새 접근 · 장축과 평행')):
        vec=np.array(d['approach_map'])[:2];vec/=np.linalg.norm(vec);beg=center[:2]-.14*vec
        ax.annotate('',xy=center[:2],xytext=beg,arrowprops=dict(arrowstyle='->',lw=2.5,color=col));ax.text(*beg,label,color=col,fontsize=9,ha='right')
    contacts=np.array([direction['fixed_contact_map_m'],direction['moving_contact_map_m']]);ax.plot(contacts[:,0],contacts[:,1],'o-',color='#8f3a9b',lw=2,label='양옆 접촉 기준선')
    ax.set_xlim(center[0]-.16,center[0]+.14);ax.set_ylim(center[1]-.18,center[1]+.17);ax.set_aspect('equal');ax.set_title('② 집게 확대 · 위에서 본 접근 방향');ax.legend(fontsize=8,loc='upper left');ax.grid(alpha=.15);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
    ax=axs[1,0]
    project=lambda v:(np.array(v)[:,:2]-center[:2])@axis
    ax.scatter(project(points),points[:,2],s=3,alpha=.35,color='#666e79',label='물체 관측')
    for v,col,label in ((fixed,'#d26c27','고정 손가락'),(moving,'#168bba','이동 손가락')):ax.scatter(project(v[::3]),v[::3,2],s=.7,alpha=.3,color=col,label=label)
    path=[]
    for q in new['entry_review']['path_model_rad']:
        tool=arm.fk(q)[0];T=transform(goal);T[2,3]+=new['lift_travel_m'];pose=T@tool@arm.tcp_from_stock
        path.append(pose[:3,3]-pose[:3,0]*arm.jaw.center_from_fixed_m(width))
    path=np.array(path);ax.plot(project(path),path[:,2],color=newcolor,lw=3,label='마지막 4cm 접근')
    ax.axhline(scene['support_height_m'],color='#9d8765',lw=2,label='관측 탁자 높이');ax.scatter([0],[center[2]],c='#8f3a9b',s=30)
    ax.set_xlim(-.17,.13);ax.set_ylim(.30,.55);ax.set_aspect('equal');ax.set_title('③ 옆에서 본 손가락 · 접촉선은 수평');ax.set_xlabel('접촉 중심 기준 장축 거리 (m)');ax.set_ylabel('바닥 기준 높이 (m)');ax.legend(fontsize=8,loc='upper right');ax.grid(alpha=.15)
    ax=axs[1,1];ax.axis('off');relative=(goal[:2]-start[:2])@transform(start)[:2,:2];yaw=math.degrees(math.atan2(math.sin(goal[2]-start[2]),math.cos(goal[2]-start[2])))
    distance=np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1).sum();reach=new['reach_comfort'];entry=new['entry_review']
    lines=[('새 후보',f'현재 본체 기준 앞 {relative[0]*100:.1f}cm · 오른쪽 {-relative[1]*100:.1f}cm\n왼쪽 회전 {yaw:.1f}° · 리프트 최하단 +{new["lift_travel_m"]*100:.0f}cm'),('방향과 거리',f'장축과 접근 방향 오차  {old_d["approach_parallel_error_deg"]:.1f}° → {direction["approach_parallel_error_deg"]:.1f}°\n어깨–접촉점 거리  {reach["shoulder_contact_distance_m"]*100:.1f}cm\n팔 사슬 길이 대비 직선거리  {reach["extension_ratio"]*100:.0f}% (하중 검증 아님)'),('저장 좌표 방식','최종 카메라 시야: 후보 탈락 조건에서 제외\n정지 관측 좌표 + 실제 상대 이동·회전으로 변환'),('이번 확인 범위',f'저장 지도 경로 약 {distance*100:.1f}cm · 고정 리프트 외형 포함\n5관절 접근 + 손가락 STL/탁자 간섭 표본 검사\n최신 관측·추적 오차·전체 팔 경로는 실행 전 확인')]
    y=.98
    for title,bodytext in lines:
        ax.text(0,y,title,color=newcolor,fontweight='bold',fontsize=12,va='top');ax.text(0,y-.065,bodytext,fontsize=10.5,va='top',linespacing=1.35);y-=.25
    fig.text(.5,.025,'촬영 rgbd-329 · 마지막 정지 align-16 | 실물 정확도·집기 성공을 증명한 그림은 아님 | 팔 영점·높이 보정값 변경 없음',ha='center',fontsize=10,color='#5c6778')
    fig.savefig(a.review/'directional-candidate.png',dpi=150)
    summary=dict(candidate_id=new['candidate_id'],relative_forward_m=float(relative[0]),relative_left_m=float(relative[1]),relative_yaw_deg=yaw,route_translation_m=float(distance),old_direction_error_deg=old_d['approach_parallel_error_deg'],new_direction_error_deg=direction['approach_parallel_error_deg'],reach_comfort=reach,old_reach_comfort=old_reach,lift_travel_m=new['lift_travel_m'],hardware_commands=0,motion_authorized=False)
    render_arm_pose(a.trial,a.review)
    (a.review/'visual-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
if __name__=='__main__':main()
