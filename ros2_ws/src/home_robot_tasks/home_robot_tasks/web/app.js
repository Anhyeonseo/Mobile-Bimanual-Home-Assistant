'use strict';
const $=id=>document.getElementById(id);
let token='', map=null, selection=null, preview=null, active=null, timer=null, geometry=null, busy=false, generation=0;
const terminal=new Set(['SUCCEEDED','FAILED','CANCELLED','INTERRUPTED']);
const labels={SUCCEEDED:'도착했어요',FAILED:'이동을 완료하지 못했어요',CANCELLED:'취소 완료',INTERRUPTED:'서버가 재시작되어 중단됐어요',STOPPING:'정지를 확인하고 있어요',RUNNING:'이동 중',WAITING:'이동 준비 중'};
async function api(path,body,permit){
  const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:{Authorization:'Bearer '+token,...(body===undefined?{}:{'Content-Type':'application/json'}),...(permit?{'X-Request-Permit':permit}:{})},body:body===undefined?undefined:JSON.stringify(body),cache:'no-store',signal:AbortSignal.timeout(5000)});
  const data=await response.json();if(!response.ok)throw new Error(data.error||'연결을 확인해 주세요');return data;
}
function error(e){$('error').textContent=e?.message||'';}
function buttons(){const owned=!!active;$('preview').disabled=busy||owned||!selection;$('go').disabled=busy||owned||!preview;$('cancel').disabled=busy||!owned;$('angle').disabled=owned||busy;}
function draw(){
  if(!map)return;const c=$('map'),w=c.clientWidth,h=c.clientHeight,dpr=window.devicePixelRatio||1;c.width=w*dpr;c.height=h*dpr;const ctx=c.getContext('2d');ctx.scale(dpr,dpr);geometry=AlohaMap.geometry(map,w,h);
  for(const s of geometry.surfaces){ctx.beginPath();s.polygon.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.fillStyle=s.color;ctx.fill();ctx.strokeStyle='#d1dcc9';ctx.lineWidth=.5;ctx.stroke();}
  const project=p=>geometry.project((p.x-map.origin.x)/map.resolution_m,(p.y-map.origin.y)/map.resolution_m,.04);
  if(preview){ctx.beginPath();preview.path_xy.forEach((p,i)=>i?ctx.lineTo(...project(p)):ctx.moveTo(...project(p)));ctx.strokeStyle='#c59b45';ctx.lineWidth=4;ctx.lineJoin='round';ctx.stroke();}
  if(selection){const [x,y]=project(selection),a=Number($('angle').value)*Math.PI/180;const head=project({x:selection.x+.5*Math.cos(a),y:selection.y+.5*Math.sin(a)});ctx.fillStyle='#215d4c';ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.fill();ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(...head);ctx.strokeStyle='#215d4c';ctx.lineWidth=3;ctx.stroke();}
}
function request(){return {schema_version:1,operation:'navigate_to',request_id:'phone-'+crypto.randomUUID(),map_id:map.map_id,map_revision:map.revision,floor_id:map.floor_id,frame_id:map.frame_id,point:{...selection},yaw_rad:Number($('angle').value)*Math.PI/180};}
async function refresh(){
  const current=await api('/v1/map');if(map&&(current.map_id!==map.map_id||current.revision!==map.revision)){selection=null;preview=null;$('selection').textContent='지도가 바뀌었어요. 다시 선택하세요.';}
  map=current;$('map-label').textContent=map.map_id+' · '+map.floor_id+' · '+map.revision;draw();buttons();
}
async function poll(){
  const epoch=generation;
  try{const health=await api('/v1/health');if(epoch!==generation)return;if(health.active_request)active=health.active_request;
    if(active){const s=await api('/v1/requests/'+encodeURIComponent(active));if(epoch!==generation)return;$('status-title').textContent=labels[s.status]||s.status;$('status-detail').textContent=s.reason||'이동과 정지 상태를 확인하고 있어요.';if(terminal.has(s.status)&&!s.control_owned){active=null;preview=null;}}
    await refresh();error(null);
  }catch(e){if(epoch===generation){error(e);$('status-title').textContent='연결을 확인하고 있어요';$('status-detail').textContent='정지 여부를 아직 확인하지 못했어요. 연결되면 상태를 다시 조회합니다.';}}
  finally{if(epoch===generation&&token){buttons();timer=setTimeout(poll,1000);}}
}
$('login').addEventListener('submit',async event=>{event.preventDefault();if(busy)return;token=$('token').value.trim();busy=true;buttons();try{await refresh();$('token').value='';$('connect').hidden=true;$('workspace').hidden=false;draw();await poll();}catch(e){error(e);token='';}finally{busy=false;buttons();}});
$('disconnect').onclick=()=>{generation++;clearTimeout(timer);token='';map=null;selection=null;preview=null;active=null;$('workspace').hidden=true;$('connect').hidden=false;$('token').value='';$('error').textContent='연결 해제는 이동 취소가 아닙니다. 다시 연결하면 진행 상태를 확인할 수 있어요.';};
$('map').addEventListener('pointerup',event=>{if(active||busy||!geometry)return;const rect=$('map').getBoundingClientRect(),cell=AlohaMap.pick(geometry,event.clientX-rect.left,event.clientY-rect.top);if(!cell){error(new Error('이동할 바닥을 선택해 주세요.'));return;}selection=AlohaMap.world(map,cell);preview=null;$('selection').textContent=`목적지 (${selection.x.toFixed(2)}, ${selection.y.toFixed(2)}) m`;error(null);draw();buttons();});
$('angle').oninput=()=>{$('angle-label').textContent=$('angle').value+'°';preview=null;draw();buttons();};
$('preview').onclick=async()=>{busy=true;buttons();try{preview=await api('/v1/navigation/preview',request());error(null);$('status-title').textContent='경로를 확인했어요';$('status-detail').textContent='표시된 목적지와 방향을 확인하고 이동을 눌러 주세요.';draw();}catch(e){preview=null;error(e);}finally{busy=false;buttons();}};
$('go').onclick=async()=>{busy=true;buttons();const body=request();try{const p=await api('/v1/permit');const s=await api('/v1/requests',body,p.permit);active=s.request_id;preview=null;error(null);}catch(e){error(e);/* Query this identity after an uncertain reply; never automatically resubmit. */try{const s=await api('/v1/requests/'+body.request_id);active=s.control_owned?s.request_id:null;}catch(_){} }finally{busy=false;buttons();}};
$('cancel').onclick=async()=>{busy=true;buttons();try{await api('/v1/requests/'+encodeURIComponent(active)+'/cancel',{});$('status-title').textContent='정지를 확인하고 있어요';error(null);}catch(e){error(e);}finally{busy=false;buttons();}};
new ResizeObserver(draw).observe($('map'));
