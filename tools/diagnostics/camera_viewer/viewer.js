(function(){
  'use strict';
  const root=document.getElementById('d415-view-range'),get=n=>root.querySelector('[data-role="'+n+'"]');
  const d=JSON.parse(get('data').textContent),kin=globalThis.S2Kinematics,geo=globalThis.CameraViewGeometry;
  const renderer=globalThis.S2MeshRenderer.create(get('mesh'),d.robot,{mul:kin.mul,point:kin.point}),svg=get('overlay');
  const mode=get('mode'),view=get('view'),height=get('height'),pitch=get('pitch');
  for(const [i,m]of d.modes.entries()){const o=document.createElement('option');o.value=String(i);o.textContent=m.label;mode.append(o);}mode.value='0';
  function element(name,attrs,text){const e=document.createElementNS('http://www.w3.org/2000/svg',name);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text)e.textContent=text;svg.append(e);return e;}
  function draw(){
    const h=Number(height.value)/100,p=Number(pitch.value),m=d.modes[Number(mode.value)],g=geo.evaluate(d,Number(mode.value),p,h);
    d.robot.base_from_optical=g.camera;
    const state=JSON.parse(JSON.stringify(d.robot.initial)),current=kin.evaluate(d.robot,state),w=svg.clientWidth,vh=svg.clientHeight;
    if(!w||!vh)return;
    svg.setAttribute('viewBox',`0 0 ${w} ${vh}`);svg.replaceChildren();
    const side=view.value==='side',top=view.value==='top',az=side?-Math.PI/2:.72,el=top?Math.PI/2:side?0:.42;
    const right=top?[0,-1,0]:[-Math.sin(az),Math.cos(az),0],up=top?[1,0,0]:[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)],depth=top?[0,0,1]:[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)];
    const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),raw=p=>[dot(p,right),-dot(p,up),dot(p,depth)];
    const bounds=[[-.3,-.35,d.ground],[.38,.35,d.ground],[-.3,.35,d.ground],[.38,-.35,d.ground],[0,0,1.15],...g.rgb,...g.depthArea,...Object.values(current.frames).map(t=>kin.point(t))];
    const b=bounds.map(raw),lo=[0,1].map(i=>Math.min(...b.map(p=>p[i]))),hi=[0,1].map(i=>Math.max(...b.map(p=>p[i])));
    const scale=Math.min((w-75)/(hi[0]-lo[0]+.1),(vh-65)/(hi[1]-lo[1]+.1));
    const project=p=>{const a=raw(p);return[w/2+(a[0]-(lo[0]+hi[0])/2)*scale,vh/2+(a[1]-(lo[1]+hi[1])/2)*scale,a[2]];};
    renderer.render(current,state,project,w,vh);
    function line(a,b,color,width=1,dash=''){const x=project(a),y=project(b);element('line',{x1:x[0],y1:x[1],x2:y[0],y2:y[1],stroke:color,'stroke-width':width,'stroke-dasharray':dash});}
    function polygon(points,color,opacity){if(points.length<3)return;element('polygon',{points:points.map(x=>project(x).slice(0,2).join(',')).join(' '),fill:color,'fill-opacity':opacity});}
    function label(point,text,dx=6,dy=-8){const a=project(point);element('text',{x:Math.max(6,Math.min(w-95,a[0]+dx)),y:Math.max(18,Math.min(vh-8,a[1]+dy)),fill:'var(--foreground)',class:'text-small'},text);}
    const camera=g.camera.slice(0,3).map(r=>r[3]);
    if(!top&&g.rgb.length){for(let i=0;i<g.rgb.length;i++)polygon([camera,g.rgb[i],g.rgb[(i+1)%g.rgb.length]],'var(--viz-series-1)',.025);}
    polygon(g.both,'var(--viz-series-1)',.18);
    for(const[points,color,dash]of [[g.rgb,'var(--viz-series-1)',''],[g.depthArea,'var(--viz-series-2)','5 4']]){
      for(let i=0;i<points.length;i++)line(points[i],points[(i+1)%points.length],color,1.6,dash);
    }
    if(!top){for(const q of g.rgb)line(camera,q,'var(--viz-series-1)',.8);}
    const maxX=Math.max(.8,...bounds.map(x=>x[0])),base=[0,0,d.ground];
    line([-.25,0,d.ground],[maxX+.04,0,d.ground],'var(--muted-foreground)');
    const tickStep=w<450?1:.5;
    for(let x=0;x<=maxX;x+=tickStep){line([x,0,d.ground],[x,0,d.ground-.025],'var(--muted-foreground)');if(!top)label([x,0,d.ground],x.toFixed(x%1?1:0)+'m',-8,20);}
    if(g.groundRange&&g.groundRange[0]>0){
      const end=g.groundRange[0];line(base,[end,0,d.ground],'var(--muted-foreground)',3,'3 3');
      if(!top&&end>.35)label([end/2,0,d.ground],'근거리 사각',-30,-12);
    }
    if(side&&g.range){line([g.range[0],0,d.ground+h],[g.range[1],0,d.ground+h],'var(--viz-series-1)',4);}
    const cp=project(camera);element('circle',{cx:cp[0],cy:cp[1],r:4,fill:'var(--foreground)'});label(camera,'D415',6,-12);
    get('height-label').textContent=h===0?'0cm · 바닥':height.value+'cm';
    get('pitch-label').textContent=p+'°'+(p===45?' · 현재 모델':' · 가상');
    const range=g.range;get('range').textContent=range?'정면 '+(h===0?'바닥':height.value+'cm 높이')+'의 RGB+깊이 범위: '+Math.round(range[0]*100)+'–'+Math.round(range[1]*100)+'cm':'이 높이의 정면 중앙선은 RGB+깊이 공통 시야 밖';
    get('details').textContent='카메라 높이 '+((camera[2]-d.ground)*100).toFixed(1)+'cm · RGB '+g.rgbFov[0].toFixed(1)+'° × '+g.rgbFov[1].toFixed(1)+'° · 깊이 최소 Z 약 '+Math.round(m.min_z_m*100)+'cm'+(m.estimated?' · RGB 광각은 공식 규격 예상':'');
    root.dataset.mode=mode.value;root.dataset.pitch=String(p);root.dataset.height=String(h);
  }
  function save(){if(window.openai?.setWidgetState)window.openai.setWidgetState({modelContent:{cameraView:d.id,mode:Number(mode.value),pitch:Number(pitch.value),surfaceHeightCm:Number(height.value)},privateContent:{view:view.value}}).catch(()=>{});}
  function restore(s){const a=s?.modelContent;if(a?.cameraView===d.id){if(d.modes[a.mode])mode.value=String(a.mode);if(Number.isFinite(a.pitch))pitch.value=Math.max(30,Math.min(85,a.pitch));if(Number.isFinite(a.surfaceHeightCm))height.value=Math.max(0,Math.min(90,a.surfaceHeightCm));}if(['side','iso','top'].includes(s?.privateContent?.view))view.value=s.privateContent.view;draw();}
  for(const c of [mode,view])c.addEventListener('change',()=>{draw();save();});
  for(const c of [height,pitch]){c.addEventListener('input',draw);c.addEventListener('change',save);}
  new ResizeObserver(draw).observe(get('mesh'));
  new MutationObserver(draw).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
  window.addEventListener('openai:set_globals',e=>restore(e.detail?.globals?.widgetState));restore(window.openai?.widgetState);
})();
