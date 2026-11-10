(function(){
  'use strict';
  const root=document.getElementById('carry-posture-view'),get=n=>root.querySelector('[data-role="'+n+'"]');
  const d=JSON.parse(get('data').textContent),{evaluate,point,mul}=globalThis.S2Kinematics;
  const renderer=globalThis.S2MeshRenderer.create(get('mesh'),d.robot,{mul,point}),svg=get('overlay');
  const select=get('case'),view=get('view');
  for(const [i,c]of d.cases.entries()){const o=document.createElement('option');o.value=String(i);o.textContent=c.label;select.append(o);}
  select.value='0';
  function state(c){const s=JSON.parse(JSON.stringify(d.state));for(let i=0;i<5;i++)s.angles.left[d.names[i]]=c.q[i];s.angles.left.gripper=c.jaw;return s;}
  function geometry(c){const current=evaluate(d.robot,state(c));const pose=c.attachment?mul(current.frames.left_tcp,c.attachment):null;return {current,points:pose?c.vertices.map(v=>point(pose,v)):[]};}
  const bounds=[];
  for(const c of d.cases){const {current,points}=geometry(c);bounds.push(...points,...Object.values(current.arms.left).map(m=>point(m)),...Object.values(current.arms.right).map(m=>point(m)));}
  function element(name,attrs,text){const e=document.createElementNS('http://www.w3.org/2000/svg',name);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;svg.append(e);return e;}
  function draw(){
    const c=d.cases[Number(select.value)],{current,points}=geometry(c),s=state(c),w=svg.clientWidth,h=svg.clientHeight;
    if(!w||!h)return;
    svg.setAttribute('viewBox',`0 0 ${w} ${h}`);svg.replaceChildren();
    const side=view.value==='arm-side',whole=view.value==='whole',az=side?Math.PI/2:.85,el=side?.08:.37;
    const right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)],depth=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)];
    const dot=(p,v)=>p.reduce((a,x,i)=>a+x*v[i],0),raw=p=>[dot(p,right),-dot(p,up),dot(p,depth)];
    const fit=whole?[...bounds,[-.25,-.35,d.ground],[.5,.35,d.ground],[0,0,1.1]]:bounds;
    const b=fit.map(raw),lo=[0,1].map(i=>Math.min(...b.map(p=>p[i]))),hi=[0,1].map(i=>Math.max(...b.map(p=>p[i])));
    const scale=Math.min((w-70)/(hi[0]-lo[0]+.08),(h-65)/(hi[1]-lo[1]+.08));
    const project=p=>{const v=raw(p);return[w/2+(v[0]-(lo[0]+hi[0])/2)*scale,h/2+(v[1]-(lo[1]+hi[1])/2)*scale,v[2]];};
    renderer.render(current,s,project,w,h);
    function line(a,b,color,width=1,dash=''){const u=project(a),v=project(b);element('line',{x1:u[0],y1:u[1],x2:v[0],y2:v[1],stroke:color,'stroke-width':width,'stroke-dasharray':dash});}
    if(points.length){
      const faces=c.faces.map(f=>({f,z:f.reduce((a,i)=>a+project(points[i])[2]/3,0)})).sort((a,b)=>a.z-b.z);
      for(const{f}of faces)element('polygon',{points:f.map(i=>project(points[i]).slice(0,2).join(',')).join(' '),fill:'var(--viz-series-2)','fill-opacity':.15});
      for(const[i,j]of c.edges)line(points[i],points[j],'var(--viz-series-2)',1.5);
      const center=c.quality.com_base_m,shoulder=c.shoulder,level=[shoulder[0],shoulder[1],center[2]];
      line(shoulder,level,'var(--muted-foreground)',1,'3 3');line(level,center,'var(--viz-series-1)',2);
      const p=project(center);element('circle',{cx:p[0],cy:p[1],r:4,fill:'var(--viz-series-2)'});
      const label=(100*c.quality.shoulder_horizontal_lever_m).toFixed(1)+' cm';
      element('text',{x:Math.min(w-65,Math.max(8,p[0]+8)),y:Math.max(20,p[1]-12),fill:'var(--foreground)',class:'text-small'},label);
      get('metric').textContent='어깨 → 물체 중심 수평 거리 '+label+' · '+(c.upright?'세운 방향 유지':'파지 방향을 고려해 조정');
    }else get('metric').textContent='공통 기준: 팔꿈치를 접고 집게를 팔 장착부 앞쪽·위쪽에 배치';
    const origin=point(current.frames.left_root),p=project(origin);
    element('circle',{cx:p[0],cy:p[1],r:3,fill:'var(--foreground)'});
    element('text',{x:Math.min(w-90,Math.max(8,p[0]+8)),y:Math.min(h-12,p[1]+18),fill:'var(--foreground)',class:'text-small'},'팔 장착부');
    root.dataset.case=c.name;root.dataset.view=view.value;
  }
  function save(){if(window.openai?.setWidgetState)window.openai.setWidgetState({modelContent:{comparison:d.id,object:d.cases[Number(select.value)].name},privateContent:{view:view.value}}).catch(()=>{});}
  function restore(saved){if(saved?.modelContent?.comparison===d.id){const i=d.cases.findIndex(c=>c.name===saved.modelContent.object);if(i>=0)select.value=String(i);}if(['arm-iso','arm-side','whole'].includes(saved?.privateContent?.view))view.value=saved.privateContent.view;draw();}
  for(const control of [select,view])control.addEventListener('change',()=>{draw();save();});
  new ResizeObserver(draw).observe(get('mesh'));
  new MutationObserver(draw).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
  window.addEventListener('openai:set_globals',e=>restore(e.detail?.globals?.widgetState));
  restore(window.openai?.widgetState);
})();
