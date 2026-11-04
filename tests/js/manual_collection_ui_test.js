'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../../tools/manual_collection.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1].replace(/initialize\(\);\s*$/, '');
const declaredIds = new Set([...html.matchAll(/<[A-Za-z][^>]*\bid="([^"]+)"[^>]*>/g)].map(match=>match[1]));
class Element {
  constructor() { this.children = []; this.value = ''; this.dataset = {}; this.listeners = {}; this.checked = false; this.classList = {toggle() {}}; }
  append(element) { this.children.push(element); }
  replaceChildren() { this.children = []; }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  querySelectorAll() { return this.children; }
  reportValidity() { return true; }
  set innerHTML(value) { throw new Error('Untrusted content must use textContent'); }
}
function environment(fetch = async () => ({ok:true, json:async () => ({phase:'idle',sessions:[]})})) {
  const elements = new Map();
  const element = id => { if (!declaredIds.has(id)) return null; if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
  const context = vm.createContext({document:{getElementById:element,createElement:()=>new Element(),addEventListener(){}},window:{addEventListener(){}},fetch,setTimeout,clearTimeout,setInterval(){},console});
  vm.runInContext(script, context);
  const run = code => vm.runInContext(code, context);
  run("config={token:'csrf',profiles:{shoulder:{motor:4,directions:{down:[-1,35,650]}}}};state={phase:'recording',sessionId:'test',joint:'shoulder'}");
  element('joint').value='shoulder';element('direction').value='down';element('percent').value=20;element('duration').value=250;
  return {run,element,context};
}

test('replay excludes future telemetry and retains distinct camera clock bounds', () => {
  const {run} = environment();
  assert.equal(run('frameTime({sourceEarliest:10,sourceLatest:10.1})'),10.1);
  assert.equal(run("eventsAt({events:[{at:9},{at:10},{at:11},{at:'9'}]},10).length"),2);
  assert.equal(run("eventsAt({events:[{at:9},{at:11}]},Infinity).length"),2);
});

test('replay deduplicates repeated output snapshots and uses text content', () => {
  const {run,element} = environment();
  run(`session={sessionId:'x',joint:'shoulder',outputBaselineSequence:0,frames:[{sourceEarliest:1}],events:[
    {kind:'command',at:1,pulse:{joint:'<img onerror=bad()>',direction:'down',percent:20,durationMs:100}},
    {kind:'http',path:'/action_outputs',at:2,response:{entries:[{sequence:1,motor:4,pwm:20,atMs:90}]}},
    {kind:'http',path:'/action_outputs',at:3,response:{entries:[{sequence:1,motor:4,pwm:20,atMs:90},{sequence:2,motor:4,pwm:0,atMs:100}]}},
    {kind:'http',path:'/status',at:3,response:{shoulderAngle:{available:true,angleDeg:234,sensorFresh:false}}}
  ]};paintEvidence(2);`);
  assert.equal(element('outputs').children.length,1);
  assert.match(element('commands').children[0].children[1].textContent,/<img onerror=bad\(\)>/);
  run('paintEvidence(Infinity)');
  assert.equal(element('outputs').children.length,2);
  assert.match(element('sensor-at-frame').textContent,/오래된 값/);
});

test('joint configuration applies M4 direction and limits without enabling controls before recording', () => {
  const {run,element} = environment();
  run("state={phase:'idle'};chooseJoint()");
  assert.deepEqual(element('direction').children.map(item=>item.value),['down']);
  assert.equal(element('percent').max,35);
  assert.equal(element('duration').max,650);
  assert.equal(element('pulse').disabled,true);
  run("state={phase:'recording'};updateControls()");
  assert.equal(element('pulse').disabled,false);
  assert.equal(element('joint').disabled,true);
});

test('a second click cannot submit a duplicate while an action response is pending', async () => {
  const calls=[];let release;
  const {run,element} = environment(async (url,options) => {
    calls.push({url,options});
    if(url==='/api/pulse')await new Promise(resolve=>{release=resolve;});
    return {ok:true,json:async()=>({phase:'recording',sessions:[]})};
  });
  const first=run("mutate('/api/pulse',{joint:'shoulder'},'done')");
  await run("mutate('/api/pulse',{joint:'shoulder'},'done')");
  assert.equal(calls.filter(call=>call.url==='/api/pulse').length,1);
  assert.equal(element('joint').disabled,true);
  assert.equal(element('pulse').disabled,true);
  assert.equal(calls[0].options.headers['X-Collection-Token'],'csrf');
  release();await first;
});

test('an ambiguous action failure sends one stop and never retries the action', async () => {
  const calls=[];
  const {run} = environment(async (url) => {
    calls.push(url);if(url==='/api/pulse')throw new Error('connection lost');
    return {ok:true,json:async()=>({phase:'failed',sessions:[]})};
  });
  await run("mutate('/api/pulse',{joint:'shoulder'},'done')");
  assert.equal(calls.filter(url=>url==='/api/pulse').length,1);
  assert.equal(calls.filter(url=>url==='/api/stop').length,1);
});

test('failed sessions remain readable and their error is displayed rather than thrown', async () => {
  const {run,element} = environment(async () => ({ok:true,json:async()=>({sessionId:'failed-record',joint:'shoulder',phase:'failed',error:'camera_stall',frames:[],events:[{kind:'result',at:10,receipt:{success:false}}]})}));
  await run("loadSession('failed-record')");
  assert.equal(element('review-quality').textContent,'오류: camera_stall');
  assert.match(element('raw-events').textContent,/success/);
  assert.equal(element('all-events').disabled,false);
  await run('refreshState()');
  assert.equal(run('disconnected'),false);
});

test('exhausted M4 command budget disables another pulse while allowing normal finish', () => {
  const {run,element} = environment();
  run("state={phase:'recording',commandCount:1};updateControls()");
  assert.equal(element('pulse').disabled,true);
  assert.equal(element('finish').disabled,false);
  assert.match(element('command-budget').textContent,/완료·저장/);
});

test('output replay excludes old ring history and preserves it only in raw evidence', () => {
  const {run,element} = environment();
  run(`session={sessionId:'x',outputBaselineSequence:10,events:[{kind:'http',at:1,path:'/action_outputs',response:{entries:[
    {sequence:10,motor:2,pwm:40,atMs:50},{sequence:11,motor:4,pwm:20,atMs:100}
  ]}}]};paintEvidence(Infinity)`);
  assert.equal(element('outputs').children.length,1);
  assert.equal(element('outputs').children[0].children[0].textContent,'11');
  assert.match(element('raw-events').textContent,/"sequence": 10/);
  run('session.outputBaselineSequence=null;paintEvidence(Infinity)');
  assert.match(element('outputs').children[0].children[0].textContent,/기준 순번 미확인/);
});

test('compact status and paged output paths remain visible in replay', () => {
  const {run,element}=environment();
  run(`session={sessionId:'compact',joint:'shoulder',outputBaselineSequence:5,frames:[{sourceEarliest:1}],events:[
    {kind:'http',path:'/collection_status',at:2,response:{shoulderAngle:{available:true,angleDeg:224,sensorFresh:true}}},
    {kind:'http',path:'/action_outputs?after=5&limit=8',at:2,response:{entries:[{sequence:6,motor:4,pwm:-51,atMs:100}]}}
  ]};paintEvidence(Infinity)`);
  assert.equal(element('outputs').children[0].children[2].textContent,'-51');
  assert.match(element('sensor-at-frame').textContent,/224/);
});
