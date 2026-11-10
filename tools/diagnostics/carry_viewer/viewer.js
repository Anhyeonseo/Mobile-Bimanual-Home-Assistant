(function(){
  'use strict';
  const root=document.getElementById('carry-capture-view'), get=n=>root.querySelector('[data-role="'+n+'"]');
  const d=JSON.parse(get('data').textContent),{evaluate,point,mul}=globalThis.S2Kinematics;
  const renderer=globalThis.S2MeshRenderer.create(get('mesh'),d.robot,{mul,point}),svg=get('overlay');
  const scenario=get('scenario'),view=get('view'),slider=get('progress');
  for(const [i,s] of d.scenarios.entries()){const o=document.createElement('option');o.value=i;o.textContent=s.label;scenario.append(o);}
  scenario.value=String(d.scenarios.length-1);
  const labels={captured:'촬영 자세',open_geometry:'집게 벌리기',pregrasp:'물체 앞 접근',approach:'직선 진입',close_geometry:'집게 닫기',lift:'2cm 들기',carry:'운반 자세 복귀'};
  function element(name,attrs,text){const e=document.createElementNS('http://www.w3.org/2000/svg',name);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;svg.append(e);return e;}
  function save(){if(window.openai?.setWidgetState)window.openai.setWidgetState({modelContent:{capture:d.capture,scenario:scenario.value,step:Number(slider.value)},privateContent:{view:view.value}}).catch(()=>{});}
  function restore(state){const a=state?.modelContent,b=state?.privateContent;if(a?.capture===d.capture&&d.scenarios[Number(a.scenario)]){scenario.value=a.scenario;slider.max=d.scenarios[Number(a.scenario)].frames.length-1;slider.value=Math.max(0,Math.min(Number(a.step)||0,Number(slider.max)));}if(['iso','top','side'].includes(b?.view))view.value=b.view;draw();}
  function draw(){
    const s=d.scenarios[Number(scenario.value)],frames=s.frames;slider.max=frames.length-1;slider.disabled=frames.length===1;
    const index=Math.min(Number(slider.value),frames.length-1),f=frames[index];
    const state={lift:s.lift,angles:JSON.parse(JSON.stringify(s.angles)),point:[0,0,0]};
    if(f.q)for(let i=0;i<5;i++)state.angles.left[d.arm_names[i]]=f.q[i];
    if(f.jaw!==undefined)state.angles.left.gripper=f.jaw;
    const current=evaluate(d.robot,state),w=svg.clientWidth,h=svg.clientHeight;
    svg.setAttribute('viewBox',`0 0 ${w} ${h}`);svg.replaceChildren();
    const az=view.value==='side'?-Math.PI/2:.75,el=view.value==='top'?1.55:view.value==='side'?.08:.37;
    const right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)],depth=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)];
    const dot=(p,v)=>p.reduce((a,x,i)=>a+x*v[i],0),raw=p=>[dot(p,right),-dot(p,up),dot(p,depth)];
    const bounds=[[-.25,-.35,d.ground],[.7,.4,1.15],[-.25,.4,1.15],[.7,-.35,d.ground],...Object.values(current.frames).map(m=>point(m)),...d.target];
    const b=bounds.map(raw),lo=[0,1].map(i=>Math.min(...b.map(p=>p[i]))),hi=[0,1].map(i=>Math.max(...b.map(p=>p[i])));
    const scale=Math.min((w-35)/(hi[0]-lo[0]),(h-30)/(hi[1]-lo[1])),cx=(lo[0]+hi[0])/2,cy=(lo[1]+hi[1])/2;
    const project=p=>{const v=raw(p);return[w/2+(v[0]-cx)*scale,h/2+(v[1]-cy)*scale,v[2]];};
    renderer.render(current,state,project,w,h);
    function line(a,b,color,width=1,dash=''){const u=project(a),v=project(b);element('line',{x1:u[0],y1:u[1],x2:v[0],y2:v[1],stroke:color,'stroke-width':width,'stroke-dasharray':dash});}
    for(const quad of d.support)for(let i=0;i<4;i++)line(quad[i],quad[(i+1)%4],'var(--border)',.7);
    for(const p of d.obstacles){const a=project(p);element('circle',{cx:a[0],cy:a[1],r:1.1,fill:'var(--muted-foreground)',opacity:.45});}
    let target=d.target;
    if(f.attached&&s.attachment){const pose=mul(current.frames.left_tcp,s.attachment);target=d.payload_corners.map(p=>point(pose,p));}
    for(const [i,j] of d.target_edges)line(target[i],target[j],'var(--viz-series-2)',2);
    const route=s.frames.filter(x=>x.tcp).map(x=>x.tcp);for(let i=1;i<route.length;i++)line(route[i-1],route[i],'var(--viz-series-1)',1.5);
    const actual=project(point(current.frames.left_tcp));element('circle',{cx:actual[0],cy:actual[1],r:4,fill:'var(--viz-series-1)'});
    for(const p of f.contacts||[]){const a=project(p);element('circle',{cx:a[0],cy:a[1],r:7,fill:'none',stroke:'var(--destructive)','stroke-width':2});}
    const center=target.reduce((a,p)=>a.map((v,i)=>v+p[i]/target.length),[0,0,0]),pos=project(center);
    element('text',{x:Math.max(5,Math.min(w-65,pos[0]+8)),y:Math.max(18,pos[1]-14),fill:'var(--foreground)',class:'text-small'},'리모컨');
    const base=[0,0,d.ground+.005];for(const [i,name]of ['앞','왼쪽','위'].entries()){const end=base.slice();end[i]+=.13;line(base,end,'var(--muted-foreground)');const a=project(end);element('text',{x:a[0]+3,y:a[1]-3,fill:'var(--foreground)',class:'text-small'},name);}
    get('stage').textContent=labels[f.label]||f.label;get('result').textContent=s.message;
    root.dataset.step=String(index);root.dataset.scenario=scenario.value;
  }
  for(const control of [scenario,view])control.addEventListener('change',()=>{slider.value=0;draw();save();});
  slider.addEventListener('input',draw);slider.addEventListener('change',save);
  new ResizeObserver(draw).observe(get('mesh'));
  new MutationObserver(draw).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
  window.addEventListener('openai:set_globals',e=>restore(e.detail?.globals?.widgetState));
  restore(window.openai?.widgetState);
})();
