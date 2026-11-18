/* Observes the existing controller UI; never generates controller commands. */
(()=>{
'use strict';
const nativeFetch=window.fetch.bind(window), pageId=crypto.randomUUID();
let sequence=0, queue=[], pending=false, dropped=0;
function mark(kind,fields={}) {
 const event={kind,pageId,sequence:++sequence,browserPerformanceMs:performance.now(),browserEpochMs:Date.now(),...fields};
 if(queue.length>=2048){dropped++;document.documentElement.dataset.intentCapture='overflow';return;}
 queue.push(event);
}
function flush(){
 if(pending||!queue.length)return;
 pending=true;const batch=queue.slice(0,64);
 nativeFetch('/__capture/events',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({events:batch,dropped})})
 .then(r=>{if(!r.ok)throw Error('capture');queue.splice(0,batch.length);document.documentElement.dataset.intentCapture=dropped?'overflow':'recording';})
 .catch(()=>{document.documentElement.dataset.intentCapture='disconnected';}).finally(()=>{pending=false;});
}
// Secret editing keeps native text/default actions, but never reaches existing
// document motor shortcuts or event telemetry. Key-up after Tab/focus change
// is also suppressed for a key that originated inside the secret region.
const secretNodes=new WeakSet(), suppressedKeys=new Set(), outsideMotorKeys=new Map();
const motorKeyIds={KeyQ:1,KeyA:1,KeyW:2,KeyS:2,KeyE:3,KeyD:3,KeyR:4,KeyF:4,KeyT:5,KeyG:5};
function secretTarget(t){return Boolean(t&&(secretNodes.has(t)||(t.closest&&t.closest('[data-intent-secret]'))));}
for(const type of ['pointerdown','pointermove','pointerup','pointercancel','mousedown','mouseup','mouseleave','touchstart','touchend','touchcancel','keydown','keyup','keypress','beforeinput','input','change','paste','copy','cut','compositionstart','compositionupdate','compositionend','click','focusin','focusout','drop']){
 document.addEventListener(type,e=>{
  const secret=secretTarget(e.target), suppressed=(type==='keyup'||type==='keypress'||(type==='keydown'&&e.repeat))&&suppressedKeys.has(e.code);
  if(type==='keydown'){
   if(secret)suppressedKeys.add(e.code);
   else if(!suppressed){suppressedKeys.delete(e.code);if(motorKeyIds[e.code])outsideMotorKeys.set(e.code,motorKeyIds[e.code]);}
  }
  if(type==='keyup'){
   const priorMotor=outsideMotorKeys.get(e.code);outsideMotorKeys.delete(e.code);suppressedKeys.delete(e.code);
   // Preserve release STOP if the key-down originated outside the secret field.
   if((secret||suppressed)&&priorMotor&&typeof window.motorStop==='function')window.motorStop(priorMotor);
  }
  // Pointer/touch releases still reach the existing joystick end handlers.
  // Telemetry listeners independently omit every secret-region event.
  if((secret||suppressed)&&['keydown','keyup','keypress'].includes(type))e.stopImmediatePropagation();
 },true);
}
function fields(e){const t=e.target;return {eventType:e.type,targetId:(t&&t.id)||'',pointerId:e.pointerId??null,clientX:e.clientX??null,clientY:e.clientY??null,isTrusted:e.isTrusted===true,value:t&&t.matches&&t.matches('input[type=range],select')?String(t.value):null};}
for(const type of ['pointerdown','pointermove','pointerup','pointercancel','mousedown','mouseup','mouseleave','touchstart','touchend','touchcancel','input','change','click'])
 document.addEventListener(type,e=>{if(secretTarget(e.target))return;if(type==='pointermove'&&!e.buttons)return;mark('dom_intent',fields(e));},true);
document.addEventListener('keydown',e=>{if(secretTarget(e.target))return;if(['KeyQ','KeyA','KeyW','KeyS','KeyE','KeyD','KeyR','KeyF','KeyT','KeyG','Escape'].includes(e.code))mark('keyboard_intent',{eventType:e.type,direction:e.code,isTrusted:e.isTrusted===true});},true);
document.addEventListener('keyup',e=>{if(secretTarget(e.target))return;if(['KeyQ','KeyA','KeyW','KeyS','KeyE','KeyD','KeyR','KeyF','KeyT','KeyG','Escape'].includes(e.code))mark('keyboard_intent',{eventType:e.type,direction:e.code,isTrusted:e.isTrusted===true});},true);
window.addEventListener('blur',()=>mark('cancel_context',{eventType:'blur'}));
document.addEventListener('visibilitychange',()=>mark('cancel_context',{eventType:document.visibilityState}));
for(const name of ['motorStart','motorStop','stopAllMotors','switchMode']){
 const original=window[name];if(typeof original!=='function')continue;
 window[name]=function(...args){mark('ui_function',{functionName:name,motorId:Number.isInteger(args[0])?args[0]:null,direction:typeof args[1]==='string'?args[1]:null,mode:name==='switchMode'?args[0]:null});return original.apply(this,args);};
}
let activeIntent=null;
window.fetch=function(input,options={}){
 const u=new URL(typeof input==='string'?input:input.url,location.href);
 if(u.origin===location.origin&&['/motor','/command'].includes(u.pathname)&&String(options.method||'GET').toUpperCase()==='POST'){
  const captureId=pageId+':'+(sequence+1), headers=new Headers(options.headers||{});headers.set('X-UI-Capture-ID',captureId);
  mark('command_dispatch',{captureId,requestSequence:activeIntent,path:u.pathname});
  return nativeFetch(input,{...options,headers});
 }
 return nativeFetch(input,options);
};
const oldCommand=window.commandFetch;
if(typeof oldCommand==='function')window.commandFetch=function(url,...args){
 // Store only known action parameters; headers, token prompts and bodies are excluded.
 const u=new URL(url,location.href), allowed=['id','action','percent','cmd','direction','duration','durationMs','speed'];
 const query={};for(const key of allowed)if(u.searchParams.has(key))query[key]=u.searchParams.get(key);
 const requestSequence=sequence+1;mark('command_intent',{path:u.pathname,query});
 try{activeIntent=requestSequence;const result=oldCommand.call(this,url,...args);activeIntent=null;return Promise.resolve(result).then(r=>{mark('command_result',{requestSequence,success:r&&r.success===true});return r;},e=>{mark('command_failure',{requestSequence});throw e;});}
 catch(e){activeIntent=null;mark('command_failure',{requestSequence});throw e;}
};
async function anchor(){const before=performance.now();try{const r=await nativeFetch('/__capture/clock?pageId='+encodeURIComponent(pageId),{cache:'no-store'});const data=await r.json();mark('clock_anchor',{browserBeforeMs:before,browserAfterMs:performance.now(),hostReceivedMonotonic:data.hostReceivedMonotonic,hostSentMonotonic:data.hostSentMonotonic,anchorId:data.anchorId});}catch(e){mark('clock_failure');}}
const banner=document.createElement('div');banner.id='intent-capture-status';banner.style.cssText='position:fixed;top:0;left:0;right:0;z-index:999999;background:#513b00;color:white;padding:6px;font:14px sans-serif';
const statusLabel=document.createElement('span'), authForm=document.createElement('form'), passwordInput=document.createElement('input'), authButton=document.createElement('button'), startButton=document.createElement('button');
authForm.dataset.intentSecret='true';authForm.style.display='inline';passwordInput.id='intent-capture-password';passwordInput.type='password';passwordInput.autocomplete='off';passwordInput.spellcheck=false;passwordInput.placeholder='명령 토큰';passwordInput.setAttribute('aria-label','명령 토큰');
authButton.id='intent-capture-auth';authButton.type='submit';authButton.textContent='인증 저장';
for(const node of [authForm,passwordInput,authButton])secretNodes.add(node);
// commandToken is the original page's lexical let binding, not a new window
// property. Original headers and invalid-token clearing continue to use it.
const originalHeaders=window.commandHeaders;
if(typeof originalHeaders==='function')window.commandHeaders=function(){return originalHeaders(false);};
authForm.onsubmit=e=>{
 e.preventDefault();
 try{commandToken=passwordInput.value.trim();passwordInput.value='';if(!window.commandHeaders(false))throw Error('missing');updateTokenStatus();statusLabel.textContent='인증 저장됨 · 수집 시작을 누르세요';}
 catch(error){commandToken='';passwordInput.value='';updateTokenStatus();statusLabel.textContent='인증 설정을 확인하세요';}
};
authForm.appendChild(passwordInput);authForm.appendChild(authButton);
startButton.id='intent-capture-start';startButton.type='button';startButton.textContent='수집 시작';
startButton.onclick=async()=>{
 if(!commandToken){statusLabel.textContent='인증을 먼저 저장하세요';return;}
 startButton.disabled=true;
 try{const r=await nativeFetch('/__capture/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pageId})});const result=await r.json();statusLabel.textContent=result.reasonText||'수집 시작 요청됨';}
 catch(error){statusLabel.textContent='수집 시작 요청 결과 불명 · 자동 재시도 없음';}
 status();
};
banner.appendChild(statusLabel);banner.appendChild(authForm);banner.appendChild(startButton);document.body.appendChild(banner);
async function status(){try{const r=await nativeFetch('/__capture/status?pageId='+encodeURIComponent(pageId));const s=await r.json();statusLabel.textContent=(s.reasonText||(s.recordingReady?'수집 연결됨 · 수동 조작 가능':'브라우저 준비 중'))+(commandToken?'':' · 인증 필요');startButton.disabled=!s.startAllowed;}catch(e){statusLabel.textContent='수집 상태 연결 실패 · STOP 확인';startButton.disabled=true;}}
setInterval(status,500);status();
mark('page_started',{instrumentationVersion:'controller_ui_intent.v1'});mark('browser_heartbeat');anchor();setInterval(anchor,5000);setInterval(flush,100);setInterval(()=>mark('browser_heartbeat'),500);
window.addEventListener('pagehide',()=>{mark('pagehide');if(queue.length)navigator.sendBeacon('/__capture/events',new Blob([JSON.stringify({events:queue.slice(0,64),dropped,unflushed:queue.length})],{type:'application/json'}));});
})();
