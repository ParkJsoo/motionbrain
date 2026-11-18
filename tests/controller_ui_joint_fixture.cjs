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
vm.runInThisContext("let commandToken='SECRET-FIXTURE';const COMMAND_TOKEN_CONFIGURED=true;let activeJointButtons={};");
function response(plan, commandId) {
  const data = {...(plan.data || {success: true}), commandId};
  return {ok: (plan.status || 200) < 400, status: plan.status || 200, json: async () => data};
}
global.fetch = async (url, options = {}) => {
  if (url === '/__capture/events') {events.push(...JSON.parse(options.body).events); return {ok: true, json: async () => ({stored: true})};}
  if (url.startsWith('/__capture/clock')) return {ok: true, json: async () => ({hostReceivedMonotonic: 1, hostSentMonotonic: 1, anchorId: 'fixtureanchor'})};
  if (url.startsWith('/__capture/status')) return {ok: true, json: async () => ({recordingReady: true, startAllowed: false})};
  const plan = plans.shift() || {}, commandId = ++nextCommandId;
  commands.push({url, headers: Object.fromEntries(new Headers(options.headers)), commandId});
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
