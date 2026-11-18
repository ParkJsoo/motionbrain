/* Observes the existing controller UI; never generates controller commands. */
(()=>{
'use strict';
const nativeFetch=window.fetch.bind(window), pageId=crypto.randomUUID();
let sequence=0, queue=[], pending=false, dropped=0;
function mark(kind,fields={}) {
 const event={kind,pageId,sequence:++sequence,browserPerformanceMs:performance.now(),browserEpochMs:Date.now(),...fields};
 if(queue.length>=2048){dropped++;document.documentElement.dataset.intentCapture='overflow';return;}
 queue.push(event);return event.sequence;
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
let activeFunction=null;const domEventContexts=new WeakMap();
function contextFor(e,seq){domEventContexts.set(e,{event:e,target:e.target,eventType:e.type,isTrusted:e.isTrusted===true,domEventSequence:seq});}
function eventContext(e){const event=e&&e.type?e:window.event;return event&&domEventContexts.get(event)||null;}
function fields(e){const t=e.target;return {eventType:e.type,targetId:(t&&t.id)||'',pointerId:e.pointerId??null,clientX:e.clientX??null,clientY:e.clientY??null,isTrusted:e.isTrusted===true,value:t&&t.matches&&t.matches('input[type=range],select')?String(t.value):null};}
for(const type of ['pointerdown','pointermove','pointerup','pointercancel','mousedown','mouseup','mouseleave','touchstart','touchend','touchcancel','input','change','click'])
 document.addEventListener(type,e=>{if(secretTarget(e.target))return;if(type==='pointermove'&&!e.buttons)return;const seq=mark('dom_intent',fields(e));contextFor(e,seq);},true);
document.addEventListener('keydown',e=>{if(secretTarget(e.target))return;if(['KeyQ','KeyA','KeyW','KeyS','KeyE','KeyD','KeyR','KeyF','KeyT','KeyG','Escape'].includes(e.code)){const seq=mark('keyboard_intent',{eventType:e.type,direction:e.code,isTrusted:e.isTrusted===true});contextFor(e,seq);}},true);
document.addEventListener('keyup',e=>{if(secretTarget(e.target))return;if(['KeyQ','KeyA','KeyW','KeyS','KeyE','KeyD','KeyR','KeyF','KeyT','KeyG','Escape'].includes(e.code)){const seq=mark('keyboard_intent',{eventType:e.type,direction:e.code,isTrusted:e.isTrusted===true});contextFor(e,seq);}},true);
window.addEventListener('blur',()=>mark('cancel_context',{eventType:'blur'}));
document.addEventListener('visibilitychange',()=>mark('cancel_context',{eventType:document.visibilityState}));
const gestures=new Map(), leaseInfo=new Map();let gestureOrdinal=0;
const jointActions=new Map(), actionTargets=new Map(), errorNotices=new Map(), failedRequests=new Map(), responseStatuses=new Map(), latestSuccess=new Map();
for(const name of ['motorStart','motorStop','stopAllMotors','switchMode','jointStart','jointStop','jointStopNow','refreshJointLease','refreshMotorLease']){
 const original=window[name];if(typeof original!=='function')continue;
 window[name]=function(...args){
  const isJoint=name.startsWith('joint')||name==='refreshJointLease',joint=isJoint?String(args[0]):null;
  const event=isJoint?(name==='jointStart'?args[2]:args[1]):(name==='motorStart'?args[2]:args[1]);
  const context=eventContext(event),target=context&&(context.event.currentTarget&&context.event.currentTarget!==document?context.event.currentTarget:context.target);
  if(name==='jointStart')jointActions.set(joint,args[1]);
  if(isJoint&&target)actionTargets.set('joint:'+joint,target);
  const action=isJoint?(name==='jointStart'?args[1]:(name==='refreshJointLease'?jointActions.get(joint):'stop')):null;
  mark('ui_function',{functionName:name,motorId:Number.isInteger(args[0])?args[0]:null,joint,action:action||null,percent:isJoint&&['jointStart','refreshJointLease'].includes(name)?Number(document.getElementById('joint-speed').value):null,direction:typeof args[1]==='string'?args[1]:null,mode:name==='switchMode'?args[0]:null,eventType:context?context.eventType:(event&&event.type)||null,isTrusted:context?context.isTrusted:event&&event.isTrusted===true,domEventSequence:context&&context.domEventSequence});
  const key=isJoint?'joint:'+joint:'motor:'+args[0],isStart=['jointStart','motorStart'].includes(name),isLease=['refreshJointLease','refreshMotorLease'].includes(name);
  const entry=isJoint?(typeof activeJointButtons==='object'&&activeJointButtons[joint]):(typeof activeMotors==='object'&&activeMotors[args[0]]);
  if(isStart&&!entry){const requestedAction=isJoint?args[1]:args[1];gestures.set(key,{id:pageId+':gesture:'+(++gestureOrdinal),key,action:requestedAction,active:true,initialAcknowledged:false,pendingRequestSequence:null});const info=leaseInfo.get(key);if(info)visibility(info,false);}
  const gesture=gestures.get(key)||null;
  if(['jointStop','jointStopNow','motorStop'].includes(name)&&gesture)gesture.active=false;
  if(isLease&&entry&&gesture&&gesture.active&&gesture.pendingRequestSequence!==null){mark('lease_coalesced',{sourceFunction:name,gestureId:gesture.id,joint,pendingRequestSequence:gesture.pendingRequestSequence,uiOutcome:'not_issued_while_lease_pending'});return;}
  const prior=activeFunction;activeFunction={name,context,target,gesture};
  try{return original.apply(this,args);}finally{activeFunction=prior;}
 };
}
function controlKey(u){return u.pathname==='/joint'?'joint:'+u.searchParams.get('joint'):u.pathname==='/motor'?'motor:'+u.searchParams.get('id'):'command';}
function actionName(u){return u.searchParams.get('action')||u.searchParams.get('cmd')||'request';}
function visibility(holder,shown){holder.hidden=!shown;holder.style.display=shown?'block':'none';}
function noticeFor(u,target){
 const key=controlKey(u),noticeKey=key+'|'+actionName(u);let record=errorNotices.get(noticeKey);
 if(!record){
  const holder=document.createElement('span'),text=document.createElement('span');holder.id='intent-error-'+key.replace(':','-')+'-'+actionName(u);holder.style.cssText='color:#ffb4b4;font-size:13px;';visibility(holder,false);holder.appendChild(text);
  record={holder,text,key,failedSequence:0};errorNotices.set(noticeKey,record);
 }
 const button=target&&target.closest?target.closest('button'):target;
 if(button&&button.insertAdjacentElement)button.insertAdjacentElement('afterend',record.holder);
 else{
  const status=u.pathname==='/joint'?document.getElementById('joint-status-'+u.searchParams.get('joint')):null,card=status&&status.closest&&status.closest('.motor-card');
  if(card)card.appendChild(record.holder);else document.body.appendChild(record.holder);
 }
 return record;
}
function displayFailure(u,data,secret,httpStatus,requestSequence,target){
 if(requestSequence<(latestSuccess.get(controlKey(u))||0))return;
 const record=noticeFor(u,target);if(requestSequence<record.failedSequence)return;record.failedSequence=requestSequence;
 let message=String(data&&(data.reasonText||data.message||data.error)||'제어기가 명령을 거절했습니다');
 for(const value of [secret,typeof commandToken==='string'?commandToken:''])if(value)message=message.split(value).join('[비밀 제외]');
 message=message.replace(/X-MotionBrain-Token\s*[:=]\s*\S+/ig,'X-MotionBrain-Token [비밀 제외]').slice(0,240);
 record.text.textContent='명령 실패 '+controlKey(u)+'/'+actionName(u)+' · '+(httpStatus?'HTTP '+httpStatus+' · ':'')+message;visibility(record.holder,true);
}
let activeIntent=null;
window.fetch=function(input,options={}){
 const u=new URL(typeof input==='string'?input:input.url,location.href);
 if(u.origin===location.origin&&['/motor','/command','/joint'].includes(u.pathname)&&String(options.method||'GET').toUpperCase()==='POST'){
  const captureId=pageId+':'+(sequence+1),headers=new Headers(options.headers||{});headers.set('X-UI-Capture-ID',captureId);
  const requestSequence=activeIntent,source=activeFunction;
  mark('command_dispatch',{captureId,requestSequence,path:u.pathname,joint:u.searchParams.get('joint'),sourceFunction:source&&source.name,gestureId:source&&source.gesture&&source.gesture.id,domEventSequence:source&&source.context&&source.context.domEventSequence});
  return nativeFetch(input,{...options,headers}).then(response=>{responseStatuses.set(requestSequence,{httpStatus:Number.isInteger(response.status)?response.status:null,captureId,proxyDisposition:response.headers&&typeof response.headers.get==='function'?response.headers.get('X-MotionBrain-UI-Proxy-Disposition'):null});return response;});
 }
 return nativeFetch(input,options);
};
function neutralLease(u,r,meta,source){
 const gesture=source&&source.gesture;
 return Boolean(source&&['refreshJointLease','refreshMotorLease'].includes(source.name)&&gesture&&gesture.initialAcknowledged&&gesture.key===controlKey(u)&&gesture.action===actionName(u)&&meta&&meta.httpStatus===409&&meta.proxyDisposition==='superseded_without_dispatch'&&r&&r.schemaVersion==='motionbrain.ui_proxy_command_disposition.v1'&&r.success===false&&r.outcome==='superseded_without_dispatch'&&r.controllerForwarded===false&&r.independentGuardsPassed===true&&r.captureId===meta.captureId&&r.commandId===undefined);
}
function displayNeutralLease(u,target,gesture){
 if(gestures.get(gesture.key)!==gesture)return;
 let holder=leaseInfo.get(gesture.key);if(!holder){holder=document.createElement('span');holder.id='intent-lease-info-'+gesture.key.replace(':','-');holder.style.cssText='color:#b8c8d8;font-size:13px;';leaseInfo.set(gesture.key,holder);}
 holder.textContent='유지 요청 미전송 · 후속 입력/STOP으로 취소되었습니다. 앞선 명령 결과와는 별개입니다.';
 const button=target&&target.closest?target.closest('button'):target;if(button&&button.insertAdjacentElement)button.insertAdjacentElement('afterend',holder);else document.body.appendChild(holder);visibility(holder,true);
}
const oldCommand=window.commandFetch;
if(typeof oldCommand==='function')window.commandFetch=function(url,...args){
 const u=new URL(url,location.href),allowed=['joint','id','action','percent','cmd','direction','duration','durationMs','speed'],query={};for(const key of allowed)if(u.searchParams.has(key))query[key]=u.searchParams.get(key);
 const requestSequence=sequence+1,key=controlKey(u),stopping=u.searchParams.get('action')==='stop'||['stop','disarm'].includes(u.searchParams.get('cmd')),secret=typeof commandToken==='string'?commandToken:'',source=activeFunction;
 const target=source&&source.target||actionTargets.get(key)||(u.pathname==='/motor'?document.getElementById('btn-'+u.searchParams.get('action')+'-'+u.searchParams.get('id')):null);
 mark('command_intent',{path:u.pathname,query,sourceFunction:source&&source.name,gestureId:source&&source.gesture&&source.gesture.id,domEventSequence:source&&source.context&&source.context.domEventSequence,relatedFailedRequestSequence:failedRequests.get(key)||null});
 const lease=source&&['refreshJointLease','refreshMotorLease'].includes(source.name)&&source.gesture;if(lease)lease.pendingRequestSequence=requestSequence;
 const clearPending=()=>{if(lease&&lease.pendingRequestSequence===requestSequence)lease.pendingRequestSequence=null;};
 const fail=(data,httpStatus)=>{if(requestSequence>=(latestSuccess.get(key)||0)&&requestSequence>=(failedRequests.get(key)||0))failedRequests.set(key,requestSequence);displayFailure(u,data,secret,httpStatus,requestSequence,target);};
 try{activeIntent=requestSequence;const result=oldCommand.call(this,url,...args);activeIntent=null;return Promise.resolve(result).then(r=>{
  const meta=responseStatuses.get(requestSequence)||null,httpStatus=meta&&meta.httpStatus||null;responseStatuses.delete(requestSequence);
  const neutral=neutralLease(u,r,meta,source);clearPending();
  mark('command_result',{requestSequence,success:r&&r.success===true,httpStatus,gestureId:source&&source.gesture&&source.gesture.id,controllerForwarded:neutral?false:undefined,uiOutcome:neutral?'lease_superseded_without_dispatch':undefined,commandId:r&&Number.isSafeInteger(r.commandId)&&r.commandId>=0?r.commandId:null,reasonCode:neutral?'superseded_not_forwarded':r&&r.success===true?null:'command_rejected'});
  if(neutral){displayNeutralLease(u,target,source.gesture);/* Compatibility only: the legacy lease callback tests success. The raw result above remains false; no controller receipt is manufactured. */return {...r,success:true,uiOnlyLeaseNoOp:true,controllerSuccess:false};}
  if(r&&r.success===true&&source&&['jointStart','motorStart'].includes(source.name)&&source.gesture)source.gesture.initialAcknowledged=true;
  if(!r||r.success!==true)fail(r,httpStatus);
  else if(!stopping){latestSuccess.set(key,Math.max(latestSuccess.get(key)||0,requestSequence));if(requestSequence>=(failedRequests.get(key)||0))failedRequests.delete(key);for(const notice of errorNotices.values())if(notice.key===key&&notice.failedSequence<=requestSequence)visibility(notice.holder,false);}
  return r;
 },e=>{clearPending();responseStatuses.delete(requestSequence);mark('command_failure',{requestSequence,reasonCode:'network_or_auth_failure'});fail({error:'인증 또는 전송 오류 · 결과를 확인하세요'},null);throw e;});}
 catch(e){activeIntent=null;clearPending();responseStatuses.delete(requestSequence);mark('command_failure',{requestSequence,reasonCode:'network_or_auth_failure'});fail({error:'인증 또는 전송 오류 · 결과를 확인하세요'},null);throw e;}
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
