(function(){
'use strict';
const root=document.getElementById('s2-lift-endpoints'),data=JSON.parse(root.querySelector('[data-role="data"]').textContent);
const mul=(a,b)=>a.map(r=>b[0].map((_,j)=>r.reduce((s,v,k)=>s+v*b[k][j],0)));
const point=(m,p=[0,0,0])=>m.slice(0,3).map(r=>r[3]+r.slice(0,3).reduce((s,v,i)=>s+v*p[i],0));
const panels=[...root.querySelectorAll('[data-end]')].map(e=>({e,svg:e.querySelector('svg'),renderer:S2MeshRenderer.create(e.querySelector('canvas'),data,{mul,point}),pose:data.endpoint_poses[e.dataset.end]}));
const measuredFloor=data.display.floor_basis==='operator_reported_underside_clearance';
root.querySelector('[data-basis]').textContent=measuredFloor?'바닥 여유 약 2.8cm 실측 반영 · 끝점은 CAD 계산':'공식 CAD 바닥 기준 · 끝점은 CAD 계산';
root.querySelector('[data-wheel-note]').textContent=measuredFloor?'바퀴는 원본 CAD 참고 위치 · 실측 바닥과의 차이는 장착 검증 전':'바퀴와 바닥: 공식 CAD 기준';
let az=.85,el=.16,zoom=false;
const ns='http://www.w3.org/2000/svg';
function draw(){for(const p of panels){
 const w=p.svg.clientWidth,h=p.svg.clientHeight;p.svg.setAttribute('viewBox',`0 0 ${w} ${h}`);p.svg.replaceChildren();
 const right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)],toward=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)];
 const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),center=zoom?p.pose.contact_m:[0,0,.55],scale=Math.min((w-40)/.68,(h-70)/1.2)*(zoom?4:1);
 const project=v=>[w/2+(dot(v,right)-dot(center,right))*scale,h/2-(dot(v,up)-dot(center,up))*scale];
 project.matrix=[[...right.map(v=>v*2*scale/w),-dot(center,right)*2*scale/w],[...up.map(v=>v*2*scale/h),-dot(center,up)*2*scale/h],[...toward.map(v=>-v/10),0],[0,0,0,1]];
 if(p.renderer)p.renderer.render({platform:p.pose.platform,contact_key:p.pose.contact_key,frames:{camera:data.base_from_optical}}, {},project,w,h);
 function element(tag,attrs,text){const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;p.svg.append(e);return e;}
 function line(a,b,color,dash=''){const x=project(a),y=project(b);element('line',{x1:x[0],y1:x[1],x2:y[0],y2:y[1],stroke:color,'stroke-width':1.2,'stroke-dasharray':dash});}
 function label(at,text,dx,dy){const a=project(at),tw=[...text].reduce((n,c)=>n+(c.charCodeAt(0)>255?12:7),0);const x=Math.max(6,Math.min(w-tw-6,a[0]+dx)),y=Math.max(16,Math.min(h-8,a[1]+dy));element('line',{x1:a[0],y1:a[1],x2:x,y2:y-5,stroke:'var(--muted-foreground)'});element('text',{x,y},text);}
 const top=point(p.pose.platform,p.pose.plate_local_point_m);
 const height=(p.pose.floor_to_platform_m*100).toFixed(1)+' cm';p.e.querySelector('[data-height]').textContent='좌 '+height+' · 우 '+(p.pose.right_floor_height_m*100).toFixed(1)+' cm';
 if(!zoom){label(top,'측정 기준 · 팔 장착판 윗면',25,-10);
 label(point(data.base_from_optical),'고정 D415',-70,-25);}
 const ground=data.display.ground_z_m,axis=[-.25,-.25];
 if(!zoom){label([.12,.12,ground],measuredFloor?'실측 바닥 기준':'CAD 바닥 기준',10,34);line([...axis,ground],[...axis,top[2]],'var(--viz-series-1)');
 for(const z of [ground,top[2]])line([axis[0]-.025,axis[1],z],[axis[0]+.025,axis[1],z],'var(--viz-series-1)');
 line(top,[...axis,top[2]],'var(--border)','3 4');
 label([...axis,(ground+top[2])/2],height,-50,0);}
 const c=project(p.pose.contact_m);element('circle',{cx:c[0],cy:c[1],r:10,fill:'none',stroke:'var(--s2-lift-contact)','stroke-width':2});element('circle',{cx:c[0],cy:c[1],r:5,fill:'var(--s2-lift-contact)',stroke:'var(--background)','stroke-width':2});
 label(p.pose.contact_m,p.e.dataset.end==='low'?'이동부 하단 ↔ 베이스 판':'슬라이더 윗끝 ↔ 두꺼운 밑단',zoom?-135:-140,zoom?65:48);
 if(!p.renderer)label([0,0,.5],'3D 표시 불가',-50,0);
}}
root.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{[az,el]={iso:[.85,.16],front:[0,0],side:[-Math.PI/2,0]}[b.dataset.view];root.querySelectorAll('[data-view]').forEach(x=>x.setAttribute('aria-pressed',String(x===b)));draw();}));
root.querySelector('[data-zoom]').addEventListener('click',e=>{zoom=!zoom;e.currentTarget.setAttribute('aria-pressed',String(zoom));draw();});
for(const p of panels){let drag=null;p.svg.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY,az,el];p.svg.setPointerCapture(e.pointerId);});p.svg.addEventListener('pointermove',e=>{if(!drag)return;az=drag[2]-(e.clientX-drag[0])*.01;el=Math.max(-1.2,Math.min(1.2,drag[3]+(e.clientY-drag[1])*.01));draw();});for(const type of ['pointerup','pointercancel'])p.svg.addEventListener(type,()=>{drag=null;});new ResizeObserver(draw).observe(p.svg);}
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);draw();
})();
