(function(){
'use strict';
const root=document.getElementById('s4-lidar-mount-review'),data=JSON.parse(root.querySelector('[data-data]').textContent),d=data.lidar_review,svg=root.querySelector('svg');
const mul=(a,b)=>a.map(r=>b[0].map((_,j)=>r.reduce((s,v,k)=>s+v*b[k][j],0))),point=(m,p=[0,0,0])=>m.slice(0,3).map(r=>r[3]+r.slice(0,3).reduce((s,v,i)=>s+v*p[i],0));
const renderer=S2MeshRenderer.create(root.querySelector('canvas'),data,{mul,point});
const ns='http://www.w3.org/2000/svg',dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),adjust={x:0,y:0,z:0,yaw:d.yaw_deg};
let az=.7,el=.4,view='mount';
function draw(){
 const w=svg.clientWidth,h=svg.clientHeight;svg.setAttribute('viewBox',`0 0 ${w} ${h}`);svg.replaceChildren();
 const m=d.mount.map((v,i)=>v+adjust[['x','y','z'][i]]/1000),yaw=adjust.yaw*Math.PI/180,c=Math.cos(yaw),s=Math.sin(yaw),model=[[c,-s,0,m[0]],[s,c,0,m[1]],[0,0,1,m[2]],[0,0,0,1]];
 let center=[.09,0,.07],span=.51;if(view==='robot'){center=[0,0,.5];span=1.25;}if(view==='scan'){center=[0,0,0];span=6.8;}
 const right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)],toward=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)],scale=Math.min(w-30,h-35)/span;
 const project=p=>[w/2+(dot(p,right)-dot(center,right))*scale,h/2-(dot(p,up)-dot(center,up))*scale,dot(p,toward)];
 project.matrix=[[...right.map(v=>v*2*scale/w),-dot(center,right)*2*scale/w],[...up.map(v=>v*2*scale/h),-dot(center,up)*2*scale/h],[...toward.map(v=>-v/10),0],[0,0,0,1]];
 if(renderer)renderer.render({platform:d.platform,frames:{camera:data.base_from_optical},extra_visuals:[{mesh:'lds01.stl',transform:model}]},{},project,w,h);
 function elem(tag,attrs,text){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;svg.append(e);return e;}
 function line(a,b,color='var(--border)',dash=''){a=project(a);b=project(b);elem('line',{x1:a[0],y1:a[1],x2:b[0],y2:b[1],stroke:color,'stroke-width':1,'stroke-dasharray':dash});}
 function label(p,text,dx,dy){const q=project(p),len=[...text].length*12,x=Math.max(8,Math.min(w-len-8,q[0]+dx)),y=Math.max(18,Math.min(h-12,q[1]+dy));elem('line',{x1:q[0],y1:q[1],x2:x,y2:y-4,stroke:'var(--muted-foreground)'});elem('text',{x,y,class:'text-small'},text);}
 const ground=data.display.ground_z_m;
 if(view==='scan'){
  for(let k=-3;k<=3;k++){line([k,-3,m[2]],[k,3,m[2]]);line([-3,k,m[2]],[3,k,m[2]]);if(k!==0)label([k,0,m[2]],`${k}m`,4,15);}
  d.ranges.forEach((r,i)=>{if(r===null)return;const a=i*Math.PI/180,p=[m[0]+r*Math.cos(a+yaw),m[1]+r*Math.sin(a+yaw),m[2]],q=project(p);elem('circle',{cx:q[0],cy:q[1],r:2.6,fill:'var(--viz-series-1)','data-tooltip':`${i}° · ${(r*100).toFixed(1)}cm`});});
  label(m,'라이다',8,-16);
 }else{
  const ring=Array.from({length:73},(_,i)=>{const a=i*Math.PI/36;return project([m[0]+.12*Math.cos(a),m[1]+.12*Math.sin(a),m[2]]).slice(0,2).join(',');}).join(' ');
  elem('polyline',{points:ring,fill:'none',stroke:'var(--viz-series-2)','stroke-width':1,'stroke-dasharray':'4 4'});
  label(m,'LDS-01 회전 중심',30,-45);label([m[0]+.046*c,m[1]+.046*s,m[2]-.016],'작은 구동 모터',40,45);
  label([m[0],m[1],d.cover_z],'앞쪽 바퀴 케이스',-180,70);
  if(view==='robot')label(point(d.platform,data.display.plate_local_point_m),'리프트 최하단',-160,-20);
  line([m[0],m[1],ground],m,'var(--viz-series-2)','3 3');
 }
 line([0,0,ground],[.32,0,ground],'var(--viz-series-1)');label([.32,0,ground],'+X 앞',5,18);
 root.querySelector('[data-readout]').textContent=`회전 중심: 앞 ${(m[0]*100).toFixed(1)}cm · 좌 ${(m[1]*100).toFixed(1)}cm · 바닥 위 ${((m[2]-ground)*100).toFixed(1)}cm | 방향 ${adjust.yaw}° (가설)`;
 for(const [k,v]of Object.entries(adjust))root.querySelector(`[data-value="${k}"]`).textContent=v+(k==='yaw'?'°':'mm');
}
root.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{view=b.dataset.view;[az,el]=view==='scan'?[-Math.PI/2,Math.PI/2]:view==='side'?[-Math.PI/2,0]:[.7,.4];root.querySelectorAll('[data-view]').forEach(x=>x.setAttribute('aria-pressed',String(x===b)));draw();}));
root.querySelectorAll('[data-adjust]').forEach(el=>el.addEventListener('input',()=>{adjust[el.dataset.adjust]=Number(el.value);draw();}));
let drag=null;svg.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY,az,el];svg.setPointerCapture(e.pointerId);});svg.addEventListener('pointermove',e=>{if(!drag)return;az=drag[2]-(e.clientX-drag[0])*.01;el=Math.max(-1.5,Math.min(1.57,drag[3]+(e.clientY-drag[1])*.01));draw();});for(const type of ['pointerup','pointercancel'])svg.addEventListener(type,()=>drag=null);
new ResizeObserver(draw).observe(svg);matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);draw();
})();
