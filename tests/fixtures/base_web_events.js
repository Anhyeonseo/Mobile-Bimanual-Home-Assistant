const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(0,'utf8');
const windows={},docs={},posts=[];let interval,failNext=false,deferNext=false,resolvePending;
function element(key){return {dataset:{key},events:{},classList:{toggle(){}},addEventListener(n,f){this.events[n]=f},setPointerCapture(){},textContent:''};}
const buttons=['w','s','a','d','q','e'].map(element),status=element(''),stop=element('');
const context={location:{hash:'#test-token'},crypto:{randomUUID:()=> 'browser-test-uuid'},
 document:{hidden:false,querySelector:q=>q==='#status'?status:stop,querySelectorAll:()=>buttons,addEventListener:(n,f)=>docs[n]=f},
 window:{addEventListener:(n,f)=>windows[n]=f},setInterval:f=>interval=f,setTimeout,clearTimeout,AbortController,
 fetch:async(path,options)=>{const data=JSON.parse(options.body);posts.push({path,...data});if(failNext){failNext=false;throw Error('lost')}if(deferNext){deferNext=false;await new Promise((resolve,reject)=>{resolvePending=resolve;options.signal.addEventListener('abort',()=>reject(Error('timeout')))})}return {ok:true,json:async()=>({active:true,status:'ready'})}}};
vm.createContext(context);vm.runInContext(source,context);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const key=k=>({key:k,repeat:false,preventDefault(){}});
(async()=>{
 await flush();assert(interval);
 windows.keydown(key('w'));await flush();assert.equal(posts.at(-1).motion,'F');
 for(let i=0;i<50;i++){interval();await flush();assert.equal(posts.at(-1).motion,'F')}
 windows.keyup(key('w'));await flush();assert.equal(posts.at(-1).motion,'Z');
 windows.keydown({...key('ㅈ'),code:'KeyW'});await flush();assert.equal(posts.at(-1).motion,'F');
 windows.keyup({...key('ㅈ'),code:'KeyW'});await flush();assert.equal(posts.at(-1).motion,'Z');
 windows.keydown(key('a'));await flush();assert.equal(posts.at(-1).motion,'L');
 windows.blur();await flush();assert.equal(posts.at(-1).motion,'Z');
 const ptr={pointerId:3,preventDefault(){}};
 buttons[0].events.pointerdown(ptr);await flush();assert.equal(posts.at(-1).motion,'F');
 buttons[0].events.pointerup(ptr);await flush();assert.equal(posts.at(-1).motion,'Z');
 // Delayed movement response: no overlapping stale heartbeat backlog. Key-up
 // queues the current zero state immediately when the one request completes.
 deferNext=true;windows.keydown(key('w'));await flush();
 const waiting=posts.length;
 for(let i=0;i<20;i++)interval();
 windows.keyup(key('w'));await flush();assert.equal(posts.length,waiting);
 resolvePending();await flush();await flush();assert.equal(posts.at(-1).motion,'Z');
 if(process.argv[2]==='timeout'){
  deferNext=true;windows.keydown(key('w'));await flush();
  await new Promise(resolve=>setTimeout(resolve,250));await flush();
 }else{
  windows.keydown(key('w'));await flush();failNext=true;interval();await flush();
 }
 const ended=posts.length;
 interval();windows.keydown(key('a'));await flush();
 assert.equal(posts.length,ended); // requires reload; no late automatic rearm
 assert(status.textContent.includes('새로고침'));

 assert(posts.filter(p=>p.path==='/control').every((p,i,a)=>i===0||p.sequence>a[i-1].sequence));
})().catch(e=>{console.error(e);process.exitCode=1});
