/* Offline DOM/transport mock executing the firmware's original joint handlers. */
const vm = require('node:vm'), fs = require('node:fs');
const config = JSON.parse(fs.readFileSync(0, 'utf8'));
const events = [], commands = [], intervals = [], created = [], listeners = {}, elements = {}, plans = [], delayed = [];
let nextCommandId = 100;
global.window = global;
global.location = {href: 'http://localhost/', origin: 'http://localhost'};
global.crypto = require('node:crypto').webcrypto;
function element(id, tagName = 'SPAN') {
  return elements[id] ??= {id, tagName, value: '60', textContent: '', style: {}, dataset: {}, children: [],
    classList: {add() {}, remove() {}}, setAttribute() {},
    appendChild(child) {this.children.push(child);},
    insertAdjacentElement(position, child) {child.anchorId = this.id;},
    closest(selector) {return selector === 'button' ? (this.tagName === 'BUTTON' ? this : null) : selector === '.motor-card' ? element('joint-card') : null;},
    matches() {return false;}};
}
global.document = {documentElement: element('root'), body: element('body'), visibilityState: 'visible',
  getElementById: element,
  createElement(tag) {const item = element('node-' + created.length, tag.toUpperCase()); created.push(item); return item;},
  addEventListener(type, fn, capture = false) {(listeners[type] ??= []).push({fn, capture});}};
global.addEventListener = (type, fn) => {(listeners[type] ??= []).push({fn, capture: false});};
global.setInterval = (fn, ms) => {intervals.push({fn, ms}); return intervals.length;};
global.clearInterval = () => {};
global.updateTokenStatus = () => {};
global.updateStatus = () => {};
global.showMessage = () => {};
global.prompt = () => {throw Error('prompt forbidden');};
global.MANUAL_LEASE_REFRESH_MS = 250;
vm.runInThisContext("let commandToken='SECRET-FIXTURE';const COMMAND_TOKEN_CONFIGURED=true;let activeJointButtons={};let activeMotors={};let currentMode='button';");
function response(plan, commandId) {
  const data = plan.superseded ? {schemaVersion:'motionbrain.ui_proxy_command_disposition.v1',success:false,outcome:'superseded_without_dispatch',controllerForwarded:false,independentGuardsPassed:true,captureId:plan.wrongCapture?'another:999':plan.captureId,reason:'superseded',...plan.bodyOverrides} : {...(plan.data || {success: true}), commandId};
  const status=plan.superseded?409:(plan.status||200);
  return {ok:status<400,status,headers:{get:name=>name==='X-MotionBrain-UI-Proxy-Disposition'&&plan.superseded&&!plan.noHeader?'superseded_without_dispatch':null},json:async()=>data};
}
global.fetch = async (url, options = {}) => {
  if (url === '/__capture/events') {events.push(...JSON.parse(options.body).events); return {ok: true, json: async () => ({stored: true})};}
  if (url.startsWith('/__capture/clock')) return {ok: true, json: async () => ({hostReceivedMonotonic: 1, hostSentMonotonic: 1, anchorId: 'fixtureanchor'})};
  if (url.startsWith('/__capture/status')) return {ok: true, json: async () => ({recordingReady: true, startAllowed: false})};
  const plan = plans.shift() || {}, commandId = plan.superseded?null:++nextCommandId;
  plan.captureId=new Headers(options.headers).get('X-UI-Capture-ID');
  commands.push({url, headers: Object.fromEntries(new Headers(options.headers)), commandId,controllerForwarded:!plan.superseded});
  if(plan.throw)throw Error('fixture network unknown');
  if (plan.delayed) return new Promise(resolve => delayed.push(() => resolve(response(plan, commandId))));
  return response(plan, commandId);
};
for (const source of config.functions) vm.runInThisContext(source);
vm.runInThisContext(config.instrumentation);
const settle = async () => {for (let i = 0; i < 12; i++) await Promise.resolve();};
async function fire(type, target, handler) {
  let stopped = false;
  const event = {type, target, currentTarget: document, isTrusted: true, pointerId: 1,
    preventDefault() {}, stopImmediatePropagation() {stopped = true;}};
  for (const entry of [...(listeners[type] || [])].sort((a, b) => Number(b.capture) - Number(a.capture))) {
    entry.fn(event);
    await settle(); // A browser checkpoint between capture and inline target handlers.
    if (stopped) break;
  }
  if (!stopped) {
    event.currentTarget = target; global.event = event;
    try {handler(event);} finally {global.event = undefined; event.currentTarget = null;}
  }
  await settle();
}
const down = element('elbow-down-button', 'BUTTON'), up = element('elbow-up-button', 'BUTTON'), stop = element('elbow-stop-button', 'BUTTON');
function notice(action) {const holder = created.find(e => e.id === 'intent-error-joint-elbow-' + action); return holder && {hidden: holder.hidden, display: holder.style.display, text: holder.children[0].textContent, anchorId: holder.anchorId};}
(async () => {
  await settle();
  if(config.scenario==='lease'){
    const snapshots={};
    await fire('mousedown',down,event=>jointStart('gripper','open',event));
    plans.push({delayed:true,superseded:true});refreshJointLease('gripper');await settle();
    const before=commands.length;refreshJointLease('gripper');refreshJointLease('gripper');await settle();snapshots.coalescedIssued=commands.length-before;
    await fire('mouseup',down,event=>jointStop('gripper',event));const beforeResolve=commands.length;
    delayed.shift()();await settle();snapshots.openReleaseExtraStops=commands.length-beforeResolve;
    snapshots.openFailureNotice=created.filter(e=>e.id.startsWith('intent-error-joint-gripper')&&!e.hidden).length;
    snapshots.openInfo=created.find(e=>e.id==='intent-lease-info-joint-gripper')?.textContent;
    await fire('mousedown',down,event=>jointStart('elbow','down',event));plans.push({delayed:true,superseded:true});refreshJointLease('elbow');await settle();
    await fire('mouseup',down,event=>jointStop('elbow',event));
    await fire('mousedown',up,event=>jointStart('elbow','up',event));plans.push({delayed:true});refreshJointLease('elbow');await settle();
    const beforeOld=commands.length;delayed.shift()();await settle();refreshJointLease('elbow');await settle();snapshots.oldGestureExtraRequests=commands.length-beforeOld;
    delayed.shift()();await settle();await fire('mouseup',up,event=>jointStop('elbow',event));
    snapshots.failures=[];
    for(const plan of [
      {status:409,data:{success:false,reason:'capture_stale',reasonText:'실제 수집 갱신 지연'}},
      {superseded:true,noHeader:true},
      {superseded:true,wrongCapture:true},
      {superseded:true,bodyOverrides:{controllerForwarded:true}},
      {status:200,data:{success:false,message:'실제 제어기 실패'}},
      {throw:true},
      {status:403,data:{success:false,error:'invalid X-MotionBrain-Token'}}
    ]){
      await fire('mousedown',down,event=>jointStart('elbow','down',event));
      const stopBefore=events.length,requestBefore=commands.length;
      plans.push(plan);refreshJointLease('elbow');await settle();
      // Drain so the original callback's STOP attempt is counted, even if auth blocks transport.
      for(let i=0;i<4;i++){intervals.find(item=>item.ms===100).fn();await settle();}
      snapshots.failures.push({plan,requests:commands.slice(requestBefore).map(c=>c.url),stopAttempts:events.slice(stopBefore).filter(e=>e.kind==='ui_function'&&e.functionName==='jointStopNow').length});
      if(!vm.runInThisContext('commandToken.length')){const input=created.find(e=>e.id==='intent-capture-password'),form=created.find(e=>e.tagName==='FORM');input.value='SECRET-FIXTURE';form.onsubmit({preventDefault(){}});}
    }
    plans.push({superseded:true});const beforeStart=commands.length;await fire('mousedown',down,event=>jointStart('elbow','down',event));snapshots.initialCanceled=commands.slice(beforeStart).map(c=>c.url);
    motorStart(3,'forward',{preventDefault(){}});await settle();plans.push({delayed:true,superseded:true});refreshMotorLease(3);await settle();const beforeMotor=commands.length;refreshMotorLease(3);await settle();motorStop(3);await settle();delayed.shift()();await settle();snapshots.motorExtra=commands.length-beforeMotor;
    for(let i=0;i<16;i++){intervals.find(item=>item.ms===100).fn();await settle();}
    console.log(JSON.stringify({events,commands,snapshots}));return;
  }
  await fire('mousedown', down, event => jointStart('elbow', 'down', event));
  element('joint-speed').value = '70'; refreshJointLease('elbow'); await settle();
  await fire('mouseup', down, event => jointStop('elbow', event));
  await fire('touchstart', up, event => jointStart('wrist', 'up', event));
  await fire('touchcancel', up, event => jointStop('wrist', event));
  await fire('click', stop, () => jointStopNow('gripper'));
  plans.push({data: {success: false, message: 'joint down rejected SECRET-FIXTURE'}});
  await fire('mousedown', down, event => jointStart('elbow', 'down', event));
  const failedDownAfterStop = notice('down');
  await fire('mousedown', down, event => jointStart('elbow', 'down', event));
  const downClearedAfterRun = notice('down');
  await fire('mouseup', down, event => jointStop('elbow', event));
  plans.push({data: {success: false, message: 'joint up rejected'}}, {data: {success: false, message: 'stop rejected'}});
  await fire('mousedown', up, event => jointStart('elbow', 'up', event));
  const upAndStopFailures = [notice('up'), notice('stop')];
  plans.push({status: 403, data: {success: false, message: 'explicit stop rejected'}});
  await fire('click', stop, () => jointStopNow('elbow'));
  const explicitStopFailure = notice('stop');
  await fire('mousedown', down, event => jointStart('elbow', 'down', event));
  const clearedByNewRun = [notice('down'), notice('up'), notice('stop')];
  await fire('mouseup', down, event => jointStop('elbow', event));
  plans.push({delayed: true});
  const oldSuccess = commandFetch('/joint?joint=elbow&action=down&percent=60', {method: 'POST'});
  plans.push({data: {success: false, message: 'newer failure'}});
  await commandFetch('/joint?joint=elbow&action=up&percent=70', {method: 'POST'});
  delayed.shift()(); await oldSuccess; await settle();
  const lateSuccessDidNotClear = notice('up');
  await commandFetch('/joint?joint=elbow&action=down&percent=60', {method: 'POST'});
  plans.push({delayed: true, data: {success: false, message: 'older failure'}});
  const oldFailure = commandFetch('/joint?joint=elbow&action=up&percent=70', {method: 'POST'});
  await commandFetch('/joint?joint=elbow&action=down&percent=60', {method: 'POST'});
  delayed.shift()(); await oldFailure; await settle();
  const lateFailureDidNotReturn = notice('up');
  for (let i = 0; i < 12; i++) {intervals.find(item => item.ms === 100).fn(); await settle();}
  console.log(JSON.stringify({events, commands, failedDownAfterStop, downClearedAfterRun, upAndStopFailures, explicitStopFailure, clearedByNewRun, lateSuccessDidNotClear, lateFailureDidNotReturn}));
})().catch(error => {console.error(error); process.exit(1);});
