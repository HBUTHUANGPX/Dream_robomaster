const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm');
const script=fs.readFileSync('web/duel/app.js','utf8');
const html=fs.readFileSync('web/duel/index.html','utf8');
function fixture(){
  const events={},elements={},calls=[];let revision=0,releaseAim=null,delayAim=false;
  const robot={hp:200,heat:0,limit:40,hits:0,shots:0,gimbal:[0,0],tracking:false,aim_error:0,flight_ms:0,fire_wait:0,fire_reason:'off',tracker_state:'LOST',switches:0};
  const state={revision:0,robots:[{...robot},{...robot}],settings:{auto_aim:false,auto_fire:false,enemy_fire:false,vision:true,protect_heat:true,invulnerable:true,speed:23,fire_rate:5,target_speed:2,target_spin:2,profile:'cooling',enemy_motion:'static',view:'overview'},time:0,paused:false,winner:null,events:[]};
  function element(id){return elements[id]??=( {id,tagName:'BUTTON',style:{},classList:{add(){},remove(){},toggle(){}},checked:false,value:'',disabled:false,setPointerCapture(){},replaceChildren(){},append(){},click(){return this.onclick?.()}} )}
  for(const match of html.matchAll(/id="([^"]+)"/g))element(match[1]);
  const document={getElementById:element,querySelectorAll:()=>[],addEventListener:(event,fn)=>events[event]=fn,createElement:()=>element('new'),createTextNode:x=>x,hidden:false};
  const context=vm.createContext({document,window:{addEventListener(){}},Date,AbortSignal,setTimeout:()=>0,clearTimeout(){},setInterval:()=>0,console,
    fetch:async(url,options)=>{
      if(!options?.method)return {ok:true,json:async()=>state};
      const name=url.split('/').pop(),args=JSON.parse(options.body);calls.push({name,args});
      if(name==='aim'&&delayAim){delayAim=false;await new Promise(resolve=>releaseAim=resolve)}
      return {ok:true,json:async()=>({ok:true,fired:true,revision:++revision})};
    }});
  vm.runInContext(script,context);
  return {context,events,elements,calls,state,eval:code=>vm.runInContext(code,context),delay:()=>delayAim=true,release:()=>releaseAim()};
}
async function flush(){for(let i=0;i<20;i++)await Promise.resolve()}
test('space fires even when last clicked button holds focus',async()=>{
 const f=fixture();await flush();let prevented=false;
 f.events.keydown({code:'Space',target:f.elements.reset,preventDefault(){prevented=true}});await flush();
 assert(prevented);assert.equal(f.calls.at(-1).name,'fire');assert(!f.calls.some(c=>c.name==='reset'));
});
test('physical key release sends stop without waiting for next timer',async()=>{
 const f=fixture();await flush();f.events.keydown({code:'KeyW',target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(f.calls.at(-1).args.vx,.5);f.events.keyup({code:'KeyW'});await flush();assert.equal(f.calls.at(-1).args.vx,0);
});
test('stale polling cannot overwrite an acknowledged slider change',async()=>{
 const f=fixture();await flush();await f.eval("setting('target_speed',3)");f.eval('syncSettings(state)');assert.equal(f.elements.targetSpeed.value,3);
});
test('pause then resume works before next poll',async()=>{
 const f=fixture();await flush();await f.elements.pause.onclick();await f.elements.pause.onclick();
 assert.deepEqual(f.calls.filter(c=>c.name==='pause').map(c=>c.args.paused),[true,false]);
});
test('center waits for outstanding drag and sends zero last',async()=>{
 const f=fixture();await flush();f.delay();f.eval("continuous('aim',{yaw:1,pitch:0})");await flush();
 const centered=f.elements.centerAim.onclick();await flush();assert.equal(f.calls.filter(c=>c.name==='aim').length,1);
 f.release();await centered;assert.equal(f.calls.filter(c=>c.name==='aim').at(-1).args.yaw,0);
});
test('reset waits for outstanding drag',async()=>{
 const f=fixture();await flush();f.delay();f.eval("continuous('aim',{yaw:1,pitch:0})");await flush();
 const reset=f.elements.reset.onclick();await flush();assert(!f.calls.some(c=>c.name==='reset'));
 f.release();await reset;assert.equal(f.calls.at(-1).name,'reset');
});
