'use strict';
const $=id=>document.getElementById(id);
let state=null,raw=false,noticeTimer,pollHealthy=false,lastState=0,drag=null,touch=null;
const keys=new Set(),pending=new Map(),editing=new Set();
const toggles={autoAim:'auto_aim',autoFire:'auto_fire',enemyFire:'enemy_fire',vision:'vision',protect:'protect_heat',invulnerable:'invulnerable'};
const ranges=[['speed','speed',' m/s'],['rate','fire_rate',' Hz'],['targetSpeed','target_speed',' m/s'],['targetSpin','target_spin',' rad/s']];
const selects={profile:'profile',motion:'enemy_motion'};
let commandTail=Promise.resolve(),pauseIntent=null;
function notice(text){$('notice').textContent=text;$('notice').style.display='block';clearTimeout(noticeTimer);noticeTimer=setTimeout(()=>$('notice').style.display='none',3200)}
async function request(name,args={}){
  const response=await fetch('/api/'+name,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(args),signal:AbortSignal.timeout(6000)});
  const value=await response.json();if(!response.ok)throw Error(value.error||'命令失败');return value;
}
function command(name,args={}){
  const result=commandTail.then(()=>request(name,args)).catch(error=>{notice(error.message);return null});
  commandTail=result;return result;
}
// One in-flight request and one latest value per continuous input stream.
const streams={drive:{busy:false,next:null},aim:{busy:false,next:null}};
function continuous(name,args){
  const stream=streams[name];stream.next=args;if(stream.busy)return;
  stream.busy=true;
  stream.promise=(async()=>{try{while(stream.next){const value=stream.next;stream.next=null;await request(name,value)}}catch(error){stream.next=null;notice(error.message)}finally{stream.busy=false}})();
}
async function setting(key,value){
  const token={value,revision:Infinity};pending.set(key,token);
  const reply=await command('settings',{[key]:value});
  if(pending.get(key)!==token)return;
  if(reply)token.revision=reply.revision;
  else{pending.delete(key);if(state)syncSettings(state)}
}
function settingValue(key){return pending.has(key)?pending.get(key).value:state?.settings[key]}
function syncSettings(s){
  for(const [key,value] of pending)if(s.revision>=value.revision)pending.delete(key);
  for(const [id,key] of Object.entries(toggles))$(id).checked=settingValue(key);
  for(const [id,key,unit] of ranges)if(!editing.has(id)){$(id).value=settingValue(key);$(id+'Value').textContent=settingValue(key)+unit}
  for(const [id,key] of Object.entries(selects))if(!editing.has(id))$(id).value=settingValue(key);
}
for(const [id,key] of Object.entries(toggles))$(id).onchange=()=>{
  if(key==='auto_aim'){drag=null;streams.aim.next=null;const value=$(id).checked;Promise.resolve(streams.aim.promise).then(()=>setting(key,value));return}
  setting(key,$(id).checked);
};
for(const [id,key,unit] of ranges){
  $(id).oninput=()=>{editing.add(id);$(id+'Value').textContent=$(id).value+unit};
  $(id).onchange=()=>{editing.delete(id);setting(key,Number($(id).value))};
  $(id).onblur=()=>editing.delete(id);
}
for(const [id,key] of Object.entries(selects))$(id).onchange=()=>setting(key,$(id).value);
async function action(id,operation){const button=$(id);if(button.disabled)return;button.disabled=true;try{await operation()}finally{button.disabled=false}}
function stop(){keys.clear();touch=null;drag=null;streams.aim.next=null;continuous('drive',{vx:0,vy:0,wz:0})}
$('fire').onclick=()=>action('fire',async()=>{const result=await command('fire');if(result&&!result.fired)notice((result.reason||'暂不能发射')+(result.wait?' · '+result.wait.toFixed(2)+' s':''))});
function isPaused(){return pauseIntent?.value??state?.paused}
async function settleControls(){await Promise.all([streams.drive.promise,streams.aim.promise])}
$('pause').onclick=()=>action('pause',async()=>{stop();if(!state)return;const value=!isPaused();pauseIntent={value,revision:Infinity};await settleControls();const reply=await command('pause',{paused:value});pauseIntent=reply?{value,revision:reply.revision}:null;$('pause').textContent=isPaused()?'继续':'暂停'});
$('reset').onclick=()=>action('reset',async()=>{stop();await settleControls();const reply=await command('reset');if(reply)pauseIntent={value:false,revision:reply.revision}});
$('stopDrive').onclick=stop;
$('centerAim').onclick=()=>action('centerAim',async()=>{stop();await settleControls();await setting('auto_aim',false);await command('aim',{yaw:0,pitch:0})});
$('overviewButton').onclick=()=>setting('view','overview');$('followButton').onclick=()=>setting('view','follow');
$('rawButton').onclick=()=>{raw=true;$('rawButton').classList.add('selected');$('annotatedButton').classList.remove('selected')};
$('annotatedButton').onclick=()=>{raw=false;$('annotatedButton').classList.add('selected');$('rawButton').classList.remove('selected')};

function show(s){if(state&&s.revision<state.revision)return;state=s;if(pauseIntent&&s.revision>=pauseIntent.revision)pauseIntent=null;$('overviewButton').classList.toggle('selected',s.settings.view==='overview');$('followButton').classList.toggle('selected',s.settings.view==='follow');const b=s.robots[0],r=s.robots[1];$('blueHp').textContent=s.settings.invulnerable?'∞':b.hp;$('redHp').textContent=s.settings.invulnerable?'∞':r.hp;$('blueBar').style.width=b.hp/2+'%';$('redBar').style.width=r.hp/2+'%';$('clock').textContent=String(Math.floor(s.time/60)).padStart(2,'0')+':'+String(Math.floor(s.time%60)).padStart(2,'0');$('result').textContent=s.winner?(s.winner==='blue'?'蓝方胜利':'红方胜利'):'';$('pause').textContent=isPaused()?'继续':'暂停';$('sceneState').textContent=s.winner?'回合结束':s.paused?'已暂停':'实时仿真 · '+s.time.toFixed(2)+' s';$('range').textContent=b.range??'—';$('aimError').textContent=b.tracking?b.aim_error.toFixed(1):'—';$('accuracy').textContent=b.shots?b.accuracy:'—';$('reproj').textContent=b.reprojection??'—';$('flight').textContent=b.tracking?b.flight_ms.toFixed(0):'—';$('shots').textContent=b.hits+' / '+b.shots;$('heatLabel').textContent='蓝方热量 '+b.heat.toFixed(1)+' / '+b.limit;$('heatBar').style.width=Math.min(100,b.heat/b.limit*100)+'%';$('heatStatus').textContent=b.locked?'超热锁枪':'每发 +10';$('redHeat').textContent='红方热量 '+r.heat.toFixed(1)+' / '+r.limit;$('redShots').textContent=r.hits+' 命中 / '+r.shots;$('trackBadge').textContent=b.locked?'图传中断':b.tracking?'已锁定':'搜索中';$('trackingStatus').textContent=b.reason;
$('numberResult').textContent=b.number?'装甲 '+b.number+' · '+(100*b.confidence).toFixed(1)+'%':'本帧无有效数字';
$('rotorState').textContent=b.tracker_state+' · ω '+(b.omega==null?'—':b.omega.toFixed(2))+' rad/s · 切板 '+b.switches+' 次';
$('rotorGeometry').textContent=b.center?'车心 ['+b.center.map(v=>v.toFixed(2)).join(', ')+'] m · 半径 '+b.radii.map(v=>(v*100).toFixed(1)).join(' / ')+' cm'+(b.tracker_state==='LOST'?'（已失效）':''):'车心与装甲半径等待初始化';$('gimbal').textContent='Yaw '+(b.gimbal[0]*180/Math.PI).toFixed(1)+'° · Pitch '+(b.gimbal[1]*180/Math.PI).toFixed(1)+'°';$('cameraHint').textContent=b.locked?'热量归零后恢复图传':s.settings.auto_aim?'自动瞄准 · 绿色为视觉检测':'拖动画面或使用方向键调整云台';$('connection').textContent='已连接 · '+(s.compute_ms??0)+' ms / 帧';syncSettings(s);$('fireStatus').textContent=b.fire_reason+(b.fire_wait>0?' · '+b.fire_wait.toFixed(2)+' s':'');$('log').replaceChildren(...s.events.slice(-5).reverse().map(e=>{const div=document.createElement('div'),t=document.createElement('time');t.textContent=e.time.toFixed(2)+'s';div.append(t,document.createTextNode(e.text));return div}));$('connectionError').style.display=s.error?'block':'none';$('connectionError').textContent=s.error?'仿真已停止：'+s.error:'';$('dot').style.background=s.error?'var(--red)':'var(--green)'}
async function poll(){
  try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('HTTP '+response.status);const value=await response.json();show(value);pollHealthy=!value.error;lastState=Date.now()}
  catch(error){pollHealthy=false;keys.clear();touch=null;drag=null;streams.drive.next=null;streams.aim.next=null;$('connection').textContent='连接中断';$('dot').style.background='var(--red)';$('connectionError').textContent='连接中断，驾驶指令已停止。';$('connectionError').style.display='block'}
  setTimeout(poll,130);
}
for(const [id,path]of [['scene',()=>'/api/scene.jpg'],['camera',()=>raw?'/api/raw.jpg':'/api/camera.jpg'],['numberRoi',()=>'/api/number.jpg']]){
  const img=$(id);const next=()=>setTimeout(()=>{if(!document.hidden)img.src=path()+'?t='+Date.now();else next()},100);img.onload=next;img.onerror=()=>setTimeout(next,500);next();
}
const keyMap={KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d',KeyQ:'q',KeyE:'e',Space:' ',ArrowUp:'arrowup',ArrowDown:'arrowdown',ArrowLeft:'arrowleft',ArrowRight:'arrowright',Escape:'escape'};
function typing(e){return ['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)||e.target.isContentEditable}
function velocity(){return touch||[.5*(keys.has('w')-keys.has('s')),.5*(keys.has('a')-keys.has('d')),.7*(keys.has('q')-keys.has('e'))]}
function sendDrive(){const v=velocity();continuous('drive',{vx:v[0],vy:v[1],wz:v[2]})}
document.addEventListener('keydown',e=>{
  const key=keyMap[e.code];if(key==='escape'){e.preventDefault();stop();return}
  if(!key||typing(e)||e.ctrlKey||e.metaKey||e.altKey)return;
  e.preventDefault();if(key===' '){if(!e.repeat)$('fire').click();return}
  if(!pollHealthy||!state||isPaused()||state.winner)return;
  keys.add(key);if(!e.repeat&&['w','a','s','d','q','e'].includes(key))sendDrive();
});
document.addEventListener('keyup',e=>{const key=keyMap[e.code];if(keys.delete(key)&&['w','a','s','d','q','e'].includes(key))sendDrive()});
window.addEventListener('blur',stop);document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});
for(const button of document.querySelectorAll('[data-drive]')){
  button.onpointerdown=e=>{e.preventDefault();if(!pollHealthy||isPaused()||state?.winner)return;button.setPointerCapture(e.pointerId);touch=button.dataset.drive.split(',').map(Number);sendDrive()};
  button.onpointerup=()=>{touch=null;sendDrive()};button.onpointercancel=stop;button.onlostpointercapture=()=>{touch=null;sendDrive()};
}
setInterval(()=>{
  if(!state||!pollHealthy||Date.now()-lastState>1000||isPaused()||state.winner)return;
  if(velocity().some(Boolean))sendDrive();
  if(!settingValue('auto_aim')&&['arrowup','arrowdown','arrowleft','arrowright'].some(k=>keys.has(k))){
    const a=state.robots[0].gimbal;
    continuous('aim',{yaw:Math.max(-6.28,Math.min(6.28,a[0]+.06*(keys.has('arrowleft')-keys.has('arrowright')))),pitch:Math.max(-.45,Math.min(.45,a[1]+.035*(keys.has('arrowup')-keys.has('arrowdown'))))});
  }
},100);
$('cameraArea').ondragstart=e=>e.preventDefault();
$('cameraArea').onpointerdown=e=>{
  if(!state||settingValue('auto_aim')||isPaused())return;e.preventDefault();$('cameraArea').setPointerCapture(e.pointerId);
  drag={x:e.clientX,y:e.clientY,angles:[...state.robots[0].gimbal]};
};
function dragAim(e){if(!drag)return;continuous('aim',{yaw:Math.max(-6.28,Math.min(6.28,drag.angles[0]-(e.clientX-drag.x)*.003)),pitch:Math.max(-.45,Math.min(.45,drag.angles[1]-(e.clientY-drag.y)*.003))})}
$('cameraArea').onpointermove=dragAim;
$('cameraArea').onpointerup=e=>{dragAim(e);drag=null};$('cameraArea').onpointercancel=()=>drag=null;$('cameraArea').onlostpointercapture=()=>drag=null;
poll();
