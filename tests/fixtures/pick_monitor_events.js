// Pure JS view/interaction unit test; no browser, network or hardware.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const html = fs.readFileSync(process.argv[2], 'utf8');
const script = html.split('<script>')[1].split('</script>')[0];
function element() {
  return {textContent:'', children:[], disabled:false, value:'리모컨 집기',
    classList:{toggle(){}}, append(...nodes){this.children.push(...nodes)},
    replaceChildren(...nodes){this.children=nodes}};
}
const elements = new Map();
const document = {getElementById(id){if(!elements.has(id))elements.set(id,element());return elements.get(id)},
                  createElement:element};
const $ = id=>document.getElementById(id);
let snapshot = {mode:'demo', phase:'READY', phase_label:'다음 요청 대기', connected:true,
  controls_enabled:true, accepting:true, blocked_reason:'', current:null,
  startup_seconds:1, server_time_s:10, events:[], history:[], last_result:null};
let offline=false, posts=[];
const context = vm.createContext({document, AbortSignal, setTimeout(){}, confirm(){return true},
  async fetch(path, options) {
    if(offline)throw Error('offline');
    if(options?.method==='POST') {posts.push({path,body:JSON.parse(options.body)});return {ok:true,json:async()=>({id:'request',requested:true})}}
    return {ok:true,json:async()=>snapshot};
  }});
vm.runInContext(script, context);
async function poll(){await vm.runInContext('poll()',context)}
(async()=>{
  await poll();
  assert.equal($('submit').disabled,false);
  assert.match($('mode').textContent,/모의 실행/);
  await $('form').onsubmit({preventDefault(){}});
  assert.deepEqual(posts[0],{path:'/api/requests',body:{text:'리모컨 집기'}});
  snapshot={...snapshot,phase:'DETECTING',phase_label:'물체 탐지',accepting:false,
    current:{started_s:5,stages:[{id:'DETECTING',label:'물체 탐지',status:'running',started_s:7}],artifacts:[]},
    last_result:{status:'SUCCEEDED',reason:'previous'}};
  await poll();
  assert.equal($('submit').disabled,true);
  assert.equal($('phase').textContent,'물체 탐지');
  assert.equal($('stages').children[0].children[2].textContent,'3.0초');
  assert.doesNotMatch($('result').textContent,/previous/);
  snapshot.events=[{at:null,message:'<img src=x onerror=alert(1)>'}];
  await poll();
  assert.equal($('logs').textContent,'<img src=x onerror=alert(1)>');
  await $('cancel').onclick();
  assert.equal(posts[1].path,'/api/cancel');
  snapshot={...snapshot,mode:'watch',controls_enabled:false,source_age_s:2,raw_status:'HOLDING'};
  await poll();
  assert.equal($('cancel').disabled,true);
  assert.equal($('request').disabled,true);
  assert.match($('elapsed').textContent,/2.0초/);
  snapshot={...snapshot,mode:'station',controls_enabled:true,accepting:true,current:null};
  $('operation').value='top-pick';
  await poll();
  assert.equal($('submit').disabled,true);
  $('supportedBottom').checked=true; $('supportedBottom').onchange();
  await $('form').onsubmit({preventDefault(){}});
  assert.deepEqual(posts.at(-1).body,{text:'리모컨 집기',operation:'top-pick',supported_empty_bottom:false,supported_empty_top:true,supported_startup_release:true});
  assert.equal($('supportedBottom').checked,false);
  snapshot={...snapshot,current:null};
  $('operation').value='observe-rise-pick';await poll();
  assert.equal($('stages').children.length,8);
  assert.equal($('stages').children[4].children[1].textContent,'리프트 상승');
  $('operation').value='top-pick';$('operation').onchange();
  assert.equal($('stages').children.length,6);
  $('operation').value='top-inspect'; $('operation').onchange();
  await $('form').onsubmit({preventDefault(){}});
  assert.equal(posts.at(-1).body.supported_empty_top,false);
  snapshot={...snapshot,reset_available:true,reset_requires_support:true};await poll();
  assert.equal($('reset').disabled,false);
  await $('reset').onclick();
  assert.deepEqual(posts.at(-1),{path:'/api/reset',body:{supported_empty:true}});
  snapshot={...snapshot,reset_available:false};await poll();
  assert.equal($('reset').disabled,true);
  snapshot={...snapshot,accepting:true,current:null,resume_source:{text:'흰색 지우개를 집어줘',prompt:'white eraser'}};await poll();
  assert.equal($('operation').value,'resume-top-pick');
  assert.equal($('stages').children.length,4);
  assert.equal($('submit').disabled,true);
  assert.match($('confirmLabel').textContent,/베이스·물체/);
  const beforeResume=posts.length;
  assert.equal(posts.length,beforeResume);
  $('supportedBottom').checked=true;$('supportedBottom').onchange();
  await $('form').onsubmit({preventDefault(){}});
  assert.equal(posts.at(-1).body.operation,'resume-top-pick');
  assert.equal(posts.at(-1).body.scene_unchanged,true);
  assert.equal(posts.at(-1).body.supported_empty_top,true);
  snapshot={...snapshot,next_test_ready:true,reset_available:true,reset_requires_support:true};
  await poll();assert.equal($('reset').textContent,'다음 시험 준비');
  await $('reset').onclick();
  assert.equal(posts.at(-1).path,'/api/reset');
  assert.equal(posts.at(-1).body.supported_empty,true);
  assert.doesNotMatch($('message').textContent,/제어기를 한 번 리셋/);
  snapshot={...snapshot,next_test_ready:false};await poll();
  assert.equal($('reset').textContent,'픽 스테이션 초기화');
  const beforeRecovery=posts.length;
  snapshot={...snapshot,current:null,last_result:{status:'REPOSITION_REVIEW_REQUIRED',reason:'no route',
    recovery:{base_reposition_candidate:true,reason:'자동 이동은 연결되지 않았습니다.'}}};await poll();
  assert.match($('result').textContent,/재배치 검토/);
  assert.equal($('recovery').hidden,false);
  assert.match($('recovery').textContent,/새 촬영/);
  assert.equal(posts.length,beforeRecovery);
  snapshot={...snapshot,last_result:null};await poll();
  assert.equal($('recovery').hidden,true);
  offline=true;await poll();
  assert.equal($('reset').disabled,true);
  assert.equal($('submit').disabled,true);
  assert.match($('connection').textContent,/끊김/);
  assert.match($('blocked').textContent,/마지막/);
  console.log('pick monitor view events passed');
})().catch(error=>{console.error(error);process.exitCode=1});
