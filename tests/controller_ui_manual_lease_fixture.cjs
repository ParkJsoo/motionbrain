/* Original firmware JS, mocked DOM/controller only. No network or real motor. */
const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');
const functions=JSON.parse(fs.readFileSync(0,'utf8')),requests=[],timers=[],elements={},docListeners={},winListeners={};
const boot='a'.repeat(32),state=Object.fromEntries([1,2,3,4,5].map(id=>[id,{generation:0,active:false,sequence:0}]));
let delaySnapshot=false,delayStart=false,delayRenew=false,failStart=false,legacy=false,snapshotReplies=[],startReplies=[],renewReplies=[];
global.window=global;global.location={href:'http://fixture/',origin:'http://fixture'};
function element(id){return elements[id]??={id,value:'60',textContent:'',style:{},classList:{contains:()=>false,add(){},remove(){}},listeners:{},addEventListener(type,fn){this.listeners[type]=fn;},getBoundingClientRect(){const n=Number(id.split('-').pop())||0;return {left:n*200,top:0,width:100,height:100,right:n*200+100,bottom:100};}};}
global.document={hidden:false,getElementById:element,addEventListener(type,fn){(docListeners[type]??=[]).push(fn);}};
global.addEventListener=(type,fn)=>{(winListeners[type]??=[]).push(fn);};
global.setInterval=(fn,ms)=>{const t={fn,ms,cleared:false};timers.push(t);return t;};global.clearInterval=t=>t.cleared=true;
global.showMessage=()=>{};global.updateStatus=()=>{};global.MANUAL_LEASE_REFRESH_MS=250;
vm.runInThisContext("let activeMotors={},activeJointButtons={},joystickActive={},activeJoysticks={},currentMode='button';let commandToken='fixture-only';const COMMAND_TOKEN_CONFIGURED=true;");
global.updateTokenStatus=()=>{};global.prompt=()=>{throw Error('prompt unexpected');};
const response=(data,ok=true)=>({ok,status:ok?200:409,json:async()=>data});
function snapshot(){return response(legacy?{motors:{}}:{schemaVersion:'motionbrain.manual_control_leases.v1',bootId:boot,ttlMs:750,motors:[1,2,3,4,5].map(motor=>({motor,generation:state[motor].generation,active:state[motor].active}))});}
const token=(motor)=>boot+'-'+motor+'-'+state[motor].generation.toString(16).padStart(8,'0');
global.fetch=async(url,options={})=>{
 requests.push({url,options});
 if(url==='/manual_leases'){if(delaySnapshot){delaySnapshot=false;return new Promise(r=>snapshotReplies.push(()=>r(snapshot())));}return snapshot();}
 const q=new URL(url,location.href).searchParams,motor=Number(q.get('id'))||({gripper:1,wrist:2,elbow:3,shoulder:4,base:5}[q.get('joint')]),s=state[motor];
 if(q.get('action')==='stop'){s.generation++;s.active=false;return response({success:true});}
 if(q.get('lease')==='start'){
  if(q.get('expectedBootId')!==boot||Number(q.get('expectedGeneration'))!==s.generation||s.active)return response({success:false},false);
  s.generation++;s.active=true;s.sequence=0;
 }else{assert.equal(q.get('lease'),'renew');if(!s.active||q.get('leaseId')!==token(motor)||Number(q.get('leaseSeq'))<=s.sequence)return response({success:false},false);s.sequence=Number(q.get('leaseSeq'));}
 const result=response(legacy?{success:true}:{success:true,manualLeaseMs:750,leaseId:token(motor),leaseGeneration:s.generation,leaseBootId:boot});
 if(failStart&&q.get('lease')==='start'){failStart=false;throw Error('unknown start result');}
 if(delayStart&&q.get('lease')==='start'){delayStart=false;return new Promise(r=>startReplies.push(()=>r(result)));}
 if(delayRenew&&q.get('lease')==='renew'){delayRenew=false;return new Promise(r=>renewReplies.push(()=>r(result)));}
 return result;
};
for(const source of functions)vm.runInThisContext(source);
const settle=async()=>{for(let i=0;i<40;i++)await Promise.resolve();};
const phase=(name)=>requests.filter(r=>new URL(r.url,location.href).searchParams.get('lease')===name);
const stops=()=>requests.filter(r=>r.url.includes('action=stop'));
const inspect=expr=>vm.runInThisContext(expr);
function fireDoc(type,extra={}){for(const f of docListeners[type]||[])f({type,preventDefault(){},...extra});}
function beginDrag(id,touchId=null){const area=element('joystick-'+id),x=id*200+50,y=20;area.listeners[touchId===null?'mousedown':'touchstart']({preventDefault(){},clientX:x,clientY:y,touches:touchId===null?undefined:[{identifier:touchId,clientX:x,clientY:y}]});}
(async()=>{
 let checks=0;
 // GET still pending at release: even a new-generation snapshot cannot issue START.
 delaySnapshot=true;motorStart(3,'forward');await settle();motorStop(3);await settle();const noStart=phase('start').length;snapshotReplies.shift()();await settle();assert.equal(phase('start').length,noStart);checks++;
 // Late successful START cannot bind its token to the next gesture.
 delayStart=true;motorStart(3,'forward');await settle();motorStop(3);await settle();motorStart(3,'reverse');await settle();const newToken=inspect('activeMotors[3].leaseId'),beforeLate=requests.length;startReplies.shift()();await settle();assert.equal(requests.length,beforeLate);assert.equal(inspect('activeMotors[3].leaseId'),newToken);motorStop(3);await settle();checks++;
 // Only one START; pending renewals coalesce and release bypasses their completion.
 motorStart(3,'forward');await settle();delayRenew=true;refreshMotorLease(3);await settle();const renewCount=phase('renew').length;refreshMotorLease(3);refreshMotorLease(3);await settle();assert.equal(phase('renew').length,renewCount);const stopCount=stops().length;motorStop(3);await settle();assert.equal(stops().length,stopCount+1);motorStart(3,'reverse');await settle();const afterNew=requests.length;renewReplies.shift()();await settle();assert.equal(requests.length,afterNew);assert(inspect('Boolean(activeMotors[3])'));motorStop(3);await settle();checks++;
 // Native aliases share a physical owner; explicit raw STOP must cancel Joint.
 jointStart('elbow','up');await settle();const beforeAlias=phase('start').length;motorStart(3,'forward');await settle();assert.equal(phase('start').length,beforeAlias);motorStopNow(3);await settle();assert(!inspect("activeJointButtons.elbow"));assert(!state[3].active);const explicitStops=stops().length;motorStopNow(3);await settle();assert.equal(stops().length,explicitStops+1);checks++;
 // Unknown start and expired renew fail closed and never synthesize a fresh START.
 failStart=true;motorStart(3,'forward');await settle();assert(!inspect('activeMotors[3]'));const unknownStarts=phase('start').length;refreshMotorLease(3);await settle();assert.equal(phase('start').length,unknownStarts);motorStart(3,'forward');await settle();state[3].active=false;state[3].generation++;refreshMotorLease(3);await settle();assert(!inspect('activeMotors[3]'));checks++;
 // Old firmware missing snapshot protocol does not receive a motor START.
 legacy=true;const oldCount=phase('start').length;motorStart(3,'forward');await settle();assert.equal(phase('start').length,oldCount);assert(!inspect('activeMotors[3]'));legacy=false;checks++;
 // Key repeat never creates a new gesture after rejection/expiry; blur stops all.
 fireDoc('keydown',{code:'KeyE',repeat:false});await settle();state[3].active=false;state[3].generation++;refreshMotorLease(3);await settle();const keyStarts=phase('start').length;fireDoc('keydown',{code:'KeyE',repeat:true});await settle();assert.equal(phase('start').length,keyStarts);fireDoc('keydown',{code:'KeyE',repeat:false});await settle();for(const f of winListeners.blur||[])f();await settle();assert(!state[3].active);checks++;
 vm.runInThisContext("currentMode='joystick'");initJoystick(3);initJoystick(4);
 // Actual joystick closure: direction reversal stops and cannot start again while dragged.
 beginDrag(3);await settle();const beforeDirection=phase('start').length;inspect('activeJoysticks[3]').updateJoystick(650,80);await settle();assert(!state[3].active);inspect('activeJoysticks[3]').updateJoystick(650,20);await settle();assert.equal(phase('start').length,beforeDirection);fireDoc('mouseup');await settle();checks++;
 // Neutral also requires release before a new START; explicit button stops joystick.
 beginDrag(3);await settle();inspect('activeJoysticks[3]').updateJoystick(650,50);await settle();const centeredStarts=phase('start').length;inspect('activeJoysticks[3]').updateJoystick(650,20);await settle();assert.equal(phase('start').length,centeredStarts);fireDoc('mouseup');await settle();beginDrag(3);await settle();motorStopNow(3);await settle();assert(!state[3].active);fireDoc('mouseup');await settle();checks++;
 // A rejected conflicting drag cannot become a fresh START after another input releases.
 jointStart('elbow','up');await settle();beginDrag(3);await settle();jointStop('elbow');await settle();const conflictStarts=phase('start').length;inspect('activeJoysticks[3]').updateJoystick(650,20);await settle();assert.equal(phase('start').length,conflictStarts);fireDoc('mouseup');await settle();beginDrag(3);await settle();assert.equal(phase('start').length,conflictStarts+1);fireDoc('mouseup');await settle();checks++;
 // Focus loss drops the drag itself; lost mouseup cannot lock out the next press.
 for(const hidden of [false,true]){
  beginDrag(3);await settle();assert(state[3].active);
  if(hidden){document.hidden=true;fireDoc('visibilitychange');}else for(const f of winListeners.blur||[])f();
  await settle();assert(!state[3].active);assert(!inspect('activeJoysticks[3]'));assert.equal(element('handle-3').style.transform,'translate(-50%, -50%)');
  const beforeLateMove=phase('start').length;fireDoc('mousemove',{clientX:650,clientY:20});await settle();assert.equal(phase('start').length,beforeLateMove);
  document.hidden=false;beginDrag(3);await settle();assert(state[3].active);assert.equal(phase('start').length,beforeLateMove+1);fireDoc('mouseup');await settle();checks++;
 }
 // Multi-touch release of one axis does not release the other.
 beginDrag(3,31);await settle();beginDrag(4,41);await settle();assert(state[3].active&&state[4].active);refreshJoystickLeases();await settle();fireDoc('touchend',{changedTouches:[{identifier:31}]});await settle();assert(!state[3].active&&state[4].active);fireDoc('touchcancel',{changedTouches:[{identifier:41}]});await settle();assert(!state[4].active);checks++;
 // Sequence numbers strictly increase within each token; no bootstrap fields on renew.
 const seqs=new Map();for(const r of phase('renew')){const q=new URL(r.url,location.href).searchParams,t=q.get('leaseId'),seq=Number(q.get('leaseSeq'));assert(seq>(seqs.get(t)||0));seqs.set(t,seq);assert(!q.has('expectedGeneration'));}
 assert(phase('start').every(r=>!new URL(r.url,location.href).searchParams.has('leaseId')));checks++;
 console.log(JSON.stringify({checks,starts:phase('start').length,renewals:phase('renew').length,stops:stops().length}));
})().catch(e=>{console.error(e.stack);process.exit(1);});
