import sys,time,json,os,stat,statistics,uuid
from pathlib import Path
sys.path.insert(0,'/tmp/motionbrain-send-age-20261015')
from learning.artifacts import ArtifactStore,_parent_fd,verify_artifacts
src=Path('/tmp/motionbrain-send-age-20261015/runs/95c43314228b470c9770fc1106ac348b')
r=json.loads((src/'run.json').read_text());m=r['results'][0]
body=(src/m['image']['artifactRef']).read_bytes()
receipt=next(json.loads(p.read_text()) for p in sorted((src/'raw').glob('*.json')) if json.loads(p.read_text()).get('path')=='/capture')
base=Path('/tmp/motionbrain-storage-profile-20261015')/uuid.uuid4().hex
rows=[]
original=os.fsync
for i in range(16):
 for mode in (('normal','prepared') if i%2==0 else ('prepared','normal')):
  store=ArtifactStore(base/f'{i}-{mode}')
  # Normal probe already has raw and images dirs from preflight/anchor.
  store.write_json('raw/preflight.json',{})
  store.save_image('anchor','source',body,receipt['sha256'])
  for k in ('measurements','persistence'):store.write_json(f'{k}/warm.json',{})
  prep=time.monotonic()
  if mode=='prepared':
   fd,_=_parent_fd(store.root,'images/0/source.jpg',create=True);os.close(fd)
  prep=(time.monotonic()-prep)*1000
  syncs=[]
  def measured(fd):
   started=time.monotonic();original(fd)
   syncs.append({'kind':'directory' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'file','ms':(time.monotonic()-started)*1000})
  os.fsync=measured
  timings={};refs=[]
  try:
   for name,fn in [('receipt',lambda:store.write_json('raw/capture.json',receipt)),
                   ('image',lambda:store.save_image('0','source',body,receipt['sha256'])),
                   ('measurement',lambda:store.write_json('measurements/0.json',m))]:
    start=time.monotonic();refs.append(fn());timings[name]=(time.monotonic()-start)*1000
  finally:os.fsync=original
  assert not verify_artifacts(store.root,refs)
  rows.append(dict(mode=mode,prepareMs=prep,stagesMs=timings,totalMs=sum(timings.values()),fsyncs=syncs))
print(json.dumps({'base':str(base),'rows':rows},indent=2))
