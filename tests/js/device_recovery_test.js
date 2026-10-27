'use strict';
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const sources = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
let passed = 0;
const config = {service:'motionbrain-dashboard', recoveryVersion:1, motionBaseUrl:'http://192.168.1.20', cameraUrl:'http://192.168.1.30'};
function env(storage={}) {
  const elements = new Map(), timers = new Map(); let id=0;
  const store = {getItem:k=>storage[k] || null, setItem:(k,v)=>{storage[k]=v;}};
  const c = {URL, AbortController, console, Date, localStorage:store, sessionStorage:store,
    setTimeout:(f,ms)=>{timers.set(++id,{f,ms});return id;},clearTimeout:i=>timers.delete(i),setInterval:()=>0,
    location:{origin:'http://192.168.1.20',hash:'',replace:u=>{c.redirect=u;}},
    document:{hidden:false,getElementById:k=>{
      if (!elements.has(k)) elements.set(k,{value:'',textContent:'',className:'',classList:{remove(){},add(){}},removeAttribute(k){delete this[k];}});
      return elements.get(k);
    },addEventListener:()=>{}},
    fetch:async()=>{throw Error('network unavailable');}};
  c.window=c;c.globalThis=c; vm.createContext(c); vm.runInContext(sources.shared,c);
  c.timers=timers;c.elements=elements;c.run=s=>vm.runInContext(s,c);return c;
}
async function test(name, fn) {await fn();passed++;console.log('PASS '+name);}
(async()=>{
  await test('registry recovers new Pi despite dead mDNS and old IP, without credentials',async()=>{
    const c=env(); const calls=[];
    c.fetch=async(url,opt)=>{calls.push(url);assert.equal(opt.credentials,'omit');assert.equal(opt.redirect,'error');assert.equal(opt.headers,undefined);
      if(url.endsWith('/vision_endpoint'))return {ok:true,json:async()=>({schema:'motionbrain.vision_endpoint.v1',available:true,remainingMs:1000,dashboardUrl:'http://192.168.1.40:8765'})};
      if(url==='http://192.168.1.40:8765/api/discovery')return {ok:true,json:async()=>config};throw Error('stale');};
    const r=await c.MotionBrainDiscovery.discover(['http://192.168.1.20'],['http://motionbrain-pi.local:8765','http://192.168.1.99:8765']);
    assert.equal(r.base,'http://192.168.1.40:8765');assert.equal(calls.length,2);
  });
  await test('expired, unrelated and external directory entries are rejected',async()=>{
    for(const entry of [
      {available:false,remainingMs:0,dashboardUrl:'http://192.168.1.40:8765'},
      {available:true,remainingMs:10,dashboardUrl:'http://evil.example'},
      {available:true,remainingMs:180001,dashboardUrl:'http://192.168.1.40:8765'},
      {available:true,remainingMs:10,dashboardUrl:'http://user:secret@192.168.1.40:8765'}]){
      const c=env();c.fetch=async()=>({ok:true,json:async()=>({schema:'motionbrain.vision_endpoint.v1',...entry})});
      assert.equal(await c.MotionBrainDiscovery.discover(['http://motionbrain.local'],[]),null);
    }
  });
  await test('hiding a tab mid-discovery prevents subsequent network requests',async()=>{
    const c=env();let count=0;
    c.fetch=async()=>{count++;c.document.hidden=true;return {ok:true,json:async()=>({schema:'motionbrain.vision_endpoint.v1',available:true,remainingMs:1000,dashboardUrl:'http://192.168.1.40:8765'})};};
    assert.equal(await c.MotionBrainDiscovery.discover(['http://motionbrain.local'],['http://motionbrain-pi.local:8765']),null);
    assert.equal(count,1);
  });
  await test('address reuse by another service never becomes a dashboard',async()=>{
    const c=env();c.fetch=async()=>({ok:true,json:async()=>({ok:true,cameraUrl:'http://192.168.1.1'})});
    assert.equal(await c.MotionBrainDiscovery.discover([],['http://192.168.1.40:8765']),null);
  });
  await test('hung discovery aborts and clears watchdog',async()=>{
    const c=env();c.fetch=(url,o)=>new Promise((_,reject)=>o.signal.addEventListener('abort',()=>reject(Error('aborted'))));
    const p=c.MotionBrainDiscovery.read('http://motionbrain.local/vision_endpoint');
    [...c.timers.values()][0].f();await assert.rejects(p,/aborted/);assert.equal(c.timers.size,0);
  });
  await test('old stored IPs migrate to automatic and follow published camera/Pi',async()=>{
    const c=env({motionbrainCameraUrl:'http://192.168.1.2',motionbrainVisionUrl:'http://192.168.1.3:8765'});c.run(sources.controller);
    c.MotionBrainDiscovery.discover=async()=>({base:'http://192.168.1.40:8765',config});
    await c.run('recoverDeviceAddresses()');
    assert.equal(c.run('cameraBaseUrl'),config.cameraUrl);assert.equal(c.run('visionBaseUrl'),'http://192.168.1.40:8765');
  });
  await test('LIVE address recovery starts exactly one replacement image',async()=>{
    const c=env({motionbrainCameraUrl:'http://192.168.1.2'});c.run(sources.controller);
    const writes=[];const img=c.document.getElementById('camera-stream');Object.defineProperty(img,'src',{configurable:true,set:v=>writes.push(v)});
    c.MotionBrainDiscovery.discover=async()=>({base:'http://192.168.1.40:8765',config});
    c.run('startCameraStream()');for(let i=0;i<8;i++)await Promise.resolve();
    assert.equal(writes.filter(v=>v.startsWith(config.cameraUrl+'/')).length,1);
    assert.equal(writes.length,2);
  });
  await test('controller hostname fallback recovers video without navigating controls',async()=>{
    const c=env();c.run(sources.controller);
    c.MotionBrainDiscovery.discover=async(controllers)=>{assert.equal(controllers[1],'http://motionbrain.local');return {base:'http://192.168.1.40:8765',config,controllerBase:'http://motionbrain.local'};};
    await c.run('recoverDeviceAddresses()');assert.equal(c.redirect,undefined);assert.equal(c.run('cameraBaseUrl'),config.cameraUrl);
  });
  await test('explicit manual endpoints are not replaced by discovered addresses',async()=>{
    const c=env({motionbrainCameraMode:'manual',motionbrainVisionMode:'manual',motionbrainCameraUrl:'http://192.168.1.2',motionbrainVisionUrl:'http://192.168.1.3:8765'});c.run(sources.controller);
    c.MotionBrainDiscovery.discover=async(controllers,bases)=>{assert.equal(controllers.length,0);assert.equal(bases[0],'http://192.168.1.3:8765');return {base:bases[0],config};};
    await c.run('recoverDeviceAddresses()');assert.equal(c.run('cameraBaseUrl'),'http://192.168.1.2');
  });
  await test('late discovery cannot overwrite a new manual selection',async()=>{
    const c=env();c.run(sources.controller);let finish;
    c.MotionBrainDiscovery.discover=()=>new Promise(r=>finish=r);
    const p=c.run('recoverDeviceAddresses()');c.run("++addressGeneration; cameraAddressMode='manual'; cameraBaseUrl='http://192.168.1.77'");
    finish({base:'http://192.168.1.40:8765',config});await p;assert.equal(c.run('cameraBaseUrl'),'http://192.168.1.77');
  });
  await test('recovery is single flight and pauses in hidden tabs',async()=>{
    const c=env();c.run(sources.controller);let finish,count=0;c.MotionBrainDiscovery.discover=()=>{count++;return new Promise(r=>finish=r);};
    const p=c.run('recoverDeviceAddresses()');await c.run('recoverDeviceAddresses()');assert.equal(count,1);finish(null);await p;
    c.document.hidden=true;await c.run('recoverDeviceAddresses()');assert.equal(count,1);
  });
  await test('snapshot remains single image even after address recovery',async()=>{
    const c=env();c.run(sources.controller);c.run("cameraMode='snapshot'");c.MotionBrainDiscovery.discover=async()=>({base:'http://192.168.1.40:8765',config});
    await c.run('recoverDeviceAddresses()');assert.equal(c.document.getElementById('camera-stream').src,undefined);
  });
  await test('controller hung image triggers retry and no motion request',async()=>{
    const c=env();c.run(sources.controller);let failed=0;c.document.getElementById('camera-stream').onerror=()=>failed++;
    c.run("loadCameraImage('http://192.168.1.30/capture')");[...c.timers.values()].find(t=>t.ms===6000).f();assert.equal(failed,1);
  });
  await test('dashboard hung image is not repeatedly cancelled by refresh interval',async()=>{
    const c=env();c.run(sources.dashboard);c.updateDetection=()=>{};c.run('refreshFrame()');const img=c.document.getElementById('cameraFrame');const src=img.src;
    c.run('refreshFrame()');assert.equal(img.src,src);assert.equal(c.timers.size,1);
    [...c.timers.values()][0].f();assert.equal(c.run('frameBusy'),false);assert.equal(img.src,undefined);
    c.run('refreshFrame()');img.onload();assert.equal(c.document.getElementById('frameConnection').textContent,'VIDEO CONNECTED');
  });
  await test('dashboard relocates after repeated failures without forwarding token/hash',async()=>{
    const c=env({motionbrainDashboardToken:'secret'});c.location.hash='#token=secret';c.run(sources.dashboard);c.run('configFailures=2');
    c.MotionBrainDiscovery.discover=async()=>({base:'http://192.168.1.40:8765',config});await c.run('recoverDashboard()');
    assert.equal(c.redirect,'http://192.168.1.40:8765/');
  });
  await test('restart at same origin does not navigate; late recovery ignored after success',async()=>{
    const c=env();c.run(sources.dashboard);c.run('configFailures=2');let finish;c.MotionBrainDiscovery.discover=()=>new Promise(r=>finish=r);
    const p=c.run('recoverDashboard()');c.run('configFailures=0');finish({base:'http://192.168.1.40:8765',config});await p;assert.equal(c.redirect,undefined);
  });
  await test('no target and connection failure have distinct labels',async()=>{
    const c=env();c.run(sources.dashboardOverlay);c.run("setTargetOverlay({detected:false,reason:'no_objects'})");assert.equal(c.document.getElementById('lockState').textContent,'NO TARGET');
    c.run("setTargetOverlay({detected:false,reason:'timeout'})");assert.equal(c.document.getElementById('lockState').textContent,'DETECTION UNAVAILABLE');
  });
  console.log(passed+' behavioral cases passed');
})().catch(e=>{console.error(e);process.exit(1);});
