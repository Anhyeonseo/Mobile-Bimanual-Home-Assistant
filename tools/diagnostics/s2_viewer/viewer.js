(function () {
  'use strict';
  const I = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
  const mul = (a,b) => a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));
  const point = (m,p=[0,0,0]) => m.slice(0,3).map(row=>row[3]+row.slice(0,3).reduce((s,v,i)=>s+v*p[i],0));
  function rot(a,q) {
    const n=Math.hypot(...a), [x,y,z]=a.map(v=>v/n), c=Math.cos(q),s=Math.sin(q),t=1-c;
    return [[t*x*x+c,t*x*y-s*z,t*x*z+s*y,0],[t*x*y+s*z,t*y*y+c,t*y*z-s*x,0],[t*x*z-s*y,t*y*z+s*x,t*z*z+c,0],[0,0,0,1]];
  }
  function evaluate(data,state) {
    const frames={base:I(),camera:data.base_from_optical}, arms={}, joints={};
    for(const side of ['left','right']) {
      const mount=data.mounts[side].map(r=>r.slice()); for(let i=0;i<3;i++)mount[i][3]+=(data.display.lift_axis_base||[0,0,1])[i]*state.lift;
      frames[side+'_root']=mount;
      const tf={base_link:mount}, pending=data.joints.slice(), axes=[];
      while(pending.length) {
        const before=pending.length;
        for(const j of pending.slice()) {
          if(!tf[j.parent])continue;
          const beforeJoint=mul(tf[j.parent],j.origin);
          tf[j.child]=j.type==='fixed'?beforeJoint:mul(beforeJoint,rot(j.axis,state.angles[side][data.names[j.name]]));
          if(j.type!=='fixed') axes.push({name:data.names[j.name],frame:tf[j.child]});
          pending.splice(pending.indexOf(j),1);
        }
        if(pending.length===before)throw Error('기구 모델의 링크 연결을 확인하세요.');
      }
      frames[side+'_tcp']=mul(tf.gripper_frame_link,data.reference_from_tcp[side]);
      arms[side]=tf; joints[side]=axes;
    }
    const p=point(data.base_from_optical,state.point), object=I();
    object[0][3]=p[0]; object[1][3]=p[1]; object[2][3]=p[2];frames.object=object;
    let platform;
    if(data.display.platform_at_lift_zero){
      platform=data.display.platform_at_lift_zero.map(r=>r.slice());
      for(let i=0;i<3;i++)platform[i][3]+=data.display.lift_axis_base[i]*state.lift;
    }
    return {frames,arms,joints,platform};
  }
  if(typeof module!=='undefined' && module.exports) {module.exports={evaluate,point,mul,rot};return;}
  const root=document.getElementById('s2-coordinate-inspector');
  if(!root)return;
  const get=name=>root.querySelector('[data-role="'+name+'"]');
  const data=JSON.parse(get('data').textContent);
  const state=JSON.parse(JSON.stringify(data.initial));
  const svg=root.querySelector('.s2-scene'), renderer=globalThis.S2MeshRenderer.create(get('robot-mesh'),data,{mul,point}), ns='http://www.w3.org/2000/svg';
  const names={base:'베이스',camera:'고정 D415',left_root:'왼팔 장착',right_root:'오른팔 장착',left_tcp:'왼팔 집게 끝',right_tcp:'오른팔 집게 끝',object:'표면점'};
  let az=.85,el=.40,dirty=false,selected='base',current,focusArms=data.display.initial_view==='arms',focusCamera=data.display.initial_view==='camera',focusBase=data.display.initial_view==='base';
  root.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.view===(data.display.initial_view||'iso'))));
  if(focusCamera){selected='camera';get('frame').value='camera';root.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.view==='camera')));}
  get('status').textContent={illustration:'실측 전 · 장착 배치 예시',simulation:'합성 보정 · 실물 미검증',measured:'실측 수락 프로필',development:'저장한 양팔 기본 자세 · 명목 좌표 연결'}[data.mode];
  get('mode').textContent=data.mode==='development'?'공식 장착 형상 + 실물 기준 자세 · 정밀 검증 S3 · 현재 실시간 상태 아님':data.mode==='illustration'?'공식 부품 형상 · 팔 장착/카메라 위치는 미보정':'좌표 프로필: '+data.calibration_id+' · 베이스 외형은 공식 CAD 참고 배치';
  get('revision').textContent='SO101 '+data.sources.find(s=>s.repository==='TheRobotStudio/SO-ARM100').commit.slice(0,12)+' · '+(data.calibration_id || '설치 보정 없음');
  get('lift').max=data.display.lift_travel_m||.64;
  get('lift').value=state.lift;
  for(const [i,c] of ['x','y','z'].entries()) {
    const input=get('point-'+c);input.min=Math.min(+input.min,state.point[i]);input.max=Math.max(+input.max,state.point[i]);input.value=state.point[i];
  }
  function projection(w,h) {
    // Fit the actual chain at both lift endpoints. The scale therefore stays
    // constant while the lift moves; the fixed camera cannot appear to drift.
    const ground=data.display.ground_z_m;
    const bounds=[[-.22,-.3,ground],[.65,.3,ground],[-.22,.3,ground],[.65,-.3,ground],[0,0,1.16]];
    for(const height of [0,data.display.lift_travel_m||.64]) {
      const at=evaluate(data,{...state,lift:height});
      for(const arm of Object.values(at.arms))for(const m of Object.values(arm))bounds.push(point(m));
      for(const m of Object.values(at.frames))bounds.push(point(m));
    }
    for(const p of [[-.12,-.08,.2],[.12,.08,.2]])bounds.push(point(data.base_from_optical,p));
    const right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)];
    const toward=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)];
    const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
    const raw=p=>[dot(p,right),-dot(p,up),dot(p,toward)];
    const b=bounds.map(raw),lo=[0,1].map(i=>Math.min(...b.map(p=>p[i]))),hi=[0,1].map(i=>Math.max(...b.map(p=>p[i])));
    let scale=Math.min((w-100)/Math.max(.3,hi[0]-lo[0]),(h-70)/Math.max(.3,hi[1]-lo[1]));
    let midx=(lo[0]+hi[0])/2,midy=(lo[1]+hi[1])/2;
    if(focusArms){const pts=['left_root','right_root','left_tcp','right_tcp'].map(k=>point(current.frames[k]));const center=[0,1,2].map(i=>pts.reduce((s,p)=>s+p[i],0)/4);const p=raw(center);midx=p[0];midy=p[1];scale*=1.8;}
    if(focusCamera){const center=point(current.frames.camera);center[2]-=.055;const p=raw(center);midx=p[0];midy=p[1];scale*=4;}
    if(focusBase){const p=raw([0,0,.075]);midx=p[0];midy=p[1];scale*=2.7;}
    const result=p=>{const v=raw(p);return [w/2+(v[0]-midx)*scale,h/2+(v[1]-midy)*scale,v[2]];};
    result.matrix=[ [...right.map(v=>v*2*scale/w),-midx*2*scale/w], [...up.map(v=>v*2*scale/h),midy*2*scale/h], [...toward.map(v=>-v/10),0], [0,0,0,1] ];
    return result;
  }
  function element(tag,attrs,text) {
    const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs||{}))e.setAttribute(k,v);if(text!=null)e.textContent=text;svg.append(e);return e;
  }
  const fmt=(v,n=1)=>(Math.abs(v)<.00000001?0:v).toFixed(n);
  function draw() {
    current=evaluate(data,state);
    const w=Math.max(280,svg.clientWidth),h=svg.clientHeight;
    svg.setAttribute('viewBox',`0 0 ${w} ${h}`);
    svg.replaceChildren();element('title',{},'ALOHA Mini 1 좌표계');element('desc',{},'관절·리프트 조절에 따른 양팔 TCP와 고정 카메라 좌표. 실물 구동 없음.');
    const project=projection(w,h), labels=[], occupied=[], segments=[];
    if(renderer)renderer.render(current,state,project,w,h);
    function line(a,b,color,width=1,dash='',opacity=1) {segments.push({a,b,color,width,dash,opacity,depth:(project(a)[2]+project(b)[2])/2});}
    function dot(p,color,r=4) {const a=project(p);element('circle',{cx:a[0],cy:a[1],r,fill:color});}
    function label(p,text,dx=8,dy=-8) {labels.push({p,text,dx,dy});}
    function arrow(m,axis,length,color,name) {
      const a=point(m),off=[0,0,0];off[axis]=length;const b=point(m,off),pa=project(a),pb=project(b);
      element('line',{x1:pa[0],y1:pa[1],x2:pb[0],y2:pb[1],stroke:color,'stroke-width':2.2});
      const vx=pb[0]-pa[0],vy=pb[1]-pa[1],n=Math.hypot(vx,vy);
      if(n>4) {const ux=vx/n,uy=vy/n; element('polygon',{points:`${pb[0]},${pb[1]} ${pb[0]-ux*7-uy*3},${pb[1]-uy*7+ux*3} ${pb[0]-ux*7+uy*3},${pb[1]-uy*7-ux*3}`,fill:color});}
      else element('circle',{cx:pa[0],cy:pa[1],r:3,fill:color});
      if(name)label(b,name,axis===1?-16:6,axis===2?-8:14);
    }
    function triad(m,length,prefix='') {['X','Y','Z'].forEach((c,i)=>arrow(m,i,length,['var(--red)','var(--green)','var(--blue)'][i],prefix+c));}
    const ground=data.display.ground_z_m;
    if(!renderer)for(let n=-3;n<=7;n++)line([n*.1,-.35,ground],[n*.1,.35,ground],'var(--border)');
    if(!renderer)for(let n=-3;n<=3;n++)line([-.3,n*.1,ground],[.7,n*.1,ground],'var(--border)');
    label([.45,-.32,ground],'바닥 · 10cm 격자',0,16);
    const l=point(current.frames.left_root),r=point(current.frames.right_root),middle=l.map((v,i)=>(v+r[i])/2);
    if(!renderer)line([middle[0],middle[1],0],middle,'var(--muted-foreground)',2,'5 5');
    if(!renderer)line(l,r,'var(--muted-foreground)',2,'5 5');
    if(!renderer)for(const side of ['left','right']) {
      const transforms=current.arms[side],color=side==='left'?'var(--viz-series-1)':'var(--viz-series-3)';
      for(const j of data.joints) {
        line(point(transforms[j.parent]),point(transforms[j.child]),color,j.type==='fixed'?4:7);
      }
      line(point(transforms.gripper_frame_link),point(current.frames[side+'_tcp']),color,3);
      const jaw=transforms.moving_jaw_so101_v1_link;
      line(point(jaw),point(jaw,[0,.035,0]),color,3);
    }
    const camera=current.frames.camera;
    const corners=[[-.12,-.08,.2],[.12,-.08,.2],[.12,.08,.2],[-.12,.08,.2]].map(p=>point(camera,p));
    if(!focusCamera && !focusBase){corners.forEach((p,i)=>{line(point(camera),p,'var(--muted-foreground)',1);line(p,corners[(i+1)%4],'var(--muted-foreground)',1);});
    line(point(camera),point(current.frames.object),'var(--viz-series-5)',1.5,'3 4');}
    if(focusBase){
      const axle=point(mul(data.surfaces.display_from_cad,data.display.cad_from_wheels[2]));
      const contact=[axle[0],axle[1],ground];
      line(axle,contact,'var(--viz-series-5)',2,'3 3');
      dot(axle,'var(--viz-series-5)',4);dot(contact,'var(--viz-series-5)',4);
      label(axle,data.display.floor_basis==='operator_reported_underside_clearance'?'CAD 축 · 실물 장착 미확인':'모터 회전축',-100,-24);label(contact,data.display.floor_basis==='operator_reported_underside_clearance'?'실측 바닥 · CAD 바퀴 미정합':'바닥 접점',-80,28);
      label([0,0,ground],(data.mode==='illustration'?'원점 → 바닥 ':'표시 기준 높이차 ')+fmt(-ground*1000,1)+' mm',60,48);
      if(data.mode==='illustration')label([0,0,0],'베이스 원점 · 세 축 중심',15,-40);
    }
    if(focusCamera && data.display.camera_mount_upper_edge_m){
      const mesh=mul(camera,data.camera_model.color_optical_from_mesh),m=data.camera_model;
      line(point(mesh,[-m.width_m/2,m.height_m/2,-m.depth_m]),point(mesh,[m.width_m/2,m.height_m/2,-m.depth_m]),'var(--viz-series-5)',2,'3 3');
      label(data.display.camera_mount_upper_edge_m,'윗면 · 경사면 상단 정렬',30,-40);
      label(point(camera),'RGB 광학 원점 · 공식 명목값',25,30);
    }
    segments.sort((a,b)=>a.depth-b.depth);
    for(const s of segments) {const a=project(s.a),b=project(s.b);element('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:s.color,'stroke-width':s.width,'stroke-linecap':'round','stroke-dasharray':s.dash,opacity:s.opacity});}
    for(const side of ['left','right']) {
      const color=side==='left'?'var(--viz-series-1)':'var(--viz-series-3)';
      if(!renderer)for(const j of current.joints[side])dot(point(j.frame),color,3);
      label(point(current.arms[side].upper_arm_link),side==='left'?'왼팔':'오른팔',side==='left'?52:-65,-25);
      dot(point(current.frames[side+'_tcp']),color,5);
      label(point(current.frames[side+'_tcp']),names[side+'_tcp'],side==='left'?18:-78,20);
    }
    dot(point(camera),'var(--foreground)',3);label(point(camera),data.mode==='illustration'?'D415 · 더듬이 정면 부착':'D415 카메라',38,focusCamera?40:-22);
    if(renderer){label([-.1,0,1.04],'고정 기둥',-85,-15);label(middle,'리프트 상판',65,18);label([0,-.1,.12],'이동 베이스',50,18);}
    dot(point(current.frames.object),'var(--viz-series-5)',5);label(point(current.frames.object),'표면점',8,16);
    dot([0,0,0],'var(--foreground)',4);label([0,0,0],'베이스',8,18);
    if(!focusBase)triad(current.frames.base,.19,'');
    if(selected!=='base' && selected!=='object')triad(current.frames[selected],.13,'');

    if(get('all-axes').checked)for(const side of ['left','right'])for(const j of current.joints[side])arrow(j.frame,2,.065,'var(--muted-foreground)','');
    const chosen=current.joints[get('arm').value].find(j=>j.name===get('joint').value);
    if(dirty) {const a=project(point(chosen.frame));element('circle',{cx:a[0],cy:a[1],r:7,fill:'none',stroke:'var(--foreground)','stroke-width':1});}
    for(const lab of labels) {
      const p=project(lab.p);if(p[0]<0||p[0]>w||p[1]<0||p[1]>h)continue;const tw=Math.max(16,[...lab.text].reduce((s,c)=>s+(c.charCodeAt(0)>255?12:7),0));
      let x=Math.max(5,Math.min(w-tw-6,p[0]+lab.dx)),y=Math.max(16,Math.min(h-8,p[1]+lab.dy));
      for(let attempt=0;attempt<14;attempt++) {
        if(!occupied.some(o=>x<o.x+o.w+4 && x+tw+4>o.x && y-12<o.y+4 && y+4>o.y-12))break;
        y=Math.min(h-8,Math.max(16,p[1]+lab.dy+(attempt%2===0?1:-1)*Math.ceil((attempt+1)/2)*17));
      }
      occupied.push({x,y,w:tw});if(lab.text.length>2 && Math.hypot(x-p[0],y-p[1])>20){const near=x>p[0]?x-3:x+tw+3;element('line',{x1:p[0],y1:p[1],x2:near,y2:y-4,stroke:'var(--muted-foreground)','stroke-width':.8});}element('text',{x,y},lab.text);
    }
    get('lift-value').textContent=fmt(state.lift*100,1)+' cm';
    get('plate-height').textContent=current.platform && data.display.plate_local_point_m
      ? '바닥 → 왼팔 장착판 윗면 약 '+fmt((point(current.platform,data.display.plate_local_point_m)[2]-ground)*100,1)+' cm · CAD + 실측 바닥 기준'
      : '';
    const selectedFrame=current.frames[selected],p=point(selectedFrame);
    get('readout').textContent=names[selected]+' · 베이스 기준 X '+fmt(p[0]*100)+' / Y '+fmt(p[1]*100)+' / Z '+fmt(p[2]*100)+' cm';
    get('pose-status').textContent=dirty?'수동 자세 미리보기 · 로봇 명령 없음':data.captured_s!=null?'기록된 촬영시각 '+data.captured_s+' s의 자세 · 현재 자세 아님':data.mode==='illustration'?'공식 모델 0° 자세 · 현재 실물 Home과 다를 수 있음':data.mode==='development'?'저장한 양팔 기본 자세 · 리프트 최저점 예시 · 현재 실시간 상태 아님':'2048에 대응하는 모델 자세 · 실제 현재값 조회 아님';
    ['x','y','z'].forEach((c,i)=>get('point-'+c+'-value').textContent=fmt(state.point[i]*100,0)+' cm');
    const tbody=get('axes-table');tbody.replaceChildren();
    ['X','Y','Z'].forEach((c,i)=>{const tr=document.createElement('tr');for(const value of [c,...selectedFrame.slice(0,3).map(row=>fmt(row[i],3))]){const td=document.createElement('td');td.textContent=value;tr.append(td);}tbody.append(tr);});
    if(selected==='object'){tbody.replaceChildren();const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.textContent='표면점: 방향축 없음';tr.append(td);tbody.append(tr);}
    root.dataset.frame=selected;root.dataset.lift=String(state.lift);
  }
  function angleControl() {
    const side=get('arm').value,joint=get('joint').value,j=data.joints.find(j=>data.names[j.name]===joint),value=state.angles[side][joint]*180/Math.PI;
    const slider=get('angle');slider.min=Math.min(Math.floor(j.limits[0]*180/Math.PI),value);slider.max=Math.max(Math.ceil(j.limits[1]*180/Math.PI),value);slider.value=value;
    get('angle-value').textContent=fmt(value,1)+'°';
  }
  get('frame').addEventListener('change',()=>{selected=get('frame').value;draw();});
  get('lift').addEventListener('input',()=>{state.lift=+get('lift').value;dirty=true;draw();});
  root.querySelectorAll('[data-lift-end]').forEach(button=>button.addEventListener('click',()=>{
    state.lift=button.dataset.liftEnd==='low'?0:+get('lift').max;
    get('lift').value=state.lift;dirty=true;draw();
  }));
  get('all-axes').addEventListener('change',draw);
  for(const name of ['arm','joint'])get(name).addEventListener('change',()=>{angleControl();draw();});
  get('angle').addEventListener('input',()=>{state.angles[get('arm').value][get('joint').value]=+get('angle').value*Math.PI/180;dirty=true;angleControl();draw();});
  for(const [i,c]of ['x','y','z'].entries())get('point-'+c).addEventListener('input',()=>{state.point[i]=+get('point-'+c).value;dirty=true;draw();});
  root.querySelectorAll('[data-view]').forEach(button=>button.addEventListener('click',()=>{
    focusArms=button.dataset.view==='arms';
    focusCamera=button.dataset.view==='camera';
    focusBase=button.dataset.view==='base';
    [az,el]={iso:[.85,.40],base:[.85,.12],camera:[.85,.40],arms:[.85,.40],front:[0,0],side:[-Math.PI/2,0],top:[0,Math.PI/2]}[button.dataset.view];
    root.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));draw();
  }));
  let drag=null;
  svg.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY,az,el];svg.setPointerCapture(e.pointerId);});
  svg.addEventListener('pointermove',e=>{if(!drag)return;az=drag[2]-(e.clientX-drag[0])*.009;el=Math.max(-1.5,Math.min(1.5,drag[3]+(e.clientY-drag[1])*.009));root.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed','false'));draw();});
  svg.addEventListener('pointerup',()=>{drag=null;});svg.addEventListener('pointercancel',()=>{drag=null;});
  angleControl();new ResizeObserver(draw).observe(svg);window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);draw();
  if(!renderer)get('mode').textContent+=' · 3D 렌더링 불가: 연결선 대체 표시';
})();
