import argparse,concurrent.futures,hashlib,json,math,pathlib,threading,time,urllib.request,serial
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--duration',type=int,default=600);a=p.parse_args()
out=pathlib.Path('/tmp/motionbrain-cam-long');out.mkdir(exist_ok=True)
log=(out/(a.label+'.jsonl')).open('w');uart=(out/(a.label+'-uart.txt')).open('w')
lock=threading.Lock();stop=threading.Event();epoch=time.monotonic();records=[]
def emit(r):
 r['elapsed']=round(time.monotonic()-epoch,3)
 with lock:records.append(r);log.write(json.dumps(r)+'\n');log.flush()
s=serial.Serial();s.port='/dev/cu.usbserial-1120';s.baudrate=115200;s.timeout=.2;s.dtr=False;s.rts=False;s.open()
def serial_read():
 while not stop.is_set():
  line=s.readline().decode('utf-8',errors='replace').strip()
  if line and not any(k in line.lower() for k in ('ssid','password','token')):
   uart.write(f'{time.monotonic()-epoch:.3f}s {line}\n');uart.flush()
t=threading.Thread(target=serial_read);t.start()
def get(kind,client='warmup'):
 begin=time.monotonic();r={'kind':kind,'client':client}
 try:
  with urllib.request.urlopen('http://192.168.219.116/'+kind,timeout=4) as response:
   body=response.read();r.update(code=response.status,bytes=len(body))
   if kind=='status':r.update(status=json.loads(body));r['ok']=response.status==200
   else:
    r.update(ok=response.status==200 and body.startswith(b'\xff\xd8') and body.endswith(b'\xff\xd9'),sha256=hashlib.sha256(body).hexdigest(),sourceFrameId=response.headers.get('X-MotionBrain-Frame-Id'))
 except Exception as ex:r.update(ok=False,error=str(ex))
 r['seconds']=round(time.monotonic()-begin,3);emit(r);return r
try:
 s.rts=True;time.sleep(.1);s.rts=False
 emit({'kind':'reset','label':a.label})
 deadline=time.monotonic()+45
 while time.monotonic()<deadline:
  r=get('status')
  if r['ok']:break
  time.sleep(1)
 else:raise RuntimeError('camera not ready within 45 seconds')
 assert r['status']['frameSize']=='qvga' and r['status']['jpegQuality']==15,r
 time.sleep(10)
 baseline=get('status');emit({'kind':'measurement_start','baseline':baseline})
 start=time.monotonic();end=start+a.duration
 def worker(kind,client,interval,offset):
  slot=0
  while True:
   due=start+offset+slot*interval
   if due>=end:return
   time.sleep(max(0,due-time.monotonic()));get(kind,client);slot+=1
   if start+offset+slot*interval<time.monotonic():
    nextslot=math.ceil((time.monotonic()-start-offset)/interval)
    emit({'kind':'missed_slots','client':client,'count':nextslot-slot});slot=nextslot
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
  jobs=[pool.submit(worker,'capture','capture1',3,0),pool.submit(worker,'capture','capture2',3,0),pool.submit(worker,'status','status',5,1.5)]
  while any(not j.done() for j in jobs):
   done=[r for r in records if r.get('client') in ('capture1','capture2','status')]
   print(json.dumps({'label':a.label,'elapsed':round(time.monotonic()-start),'requests':len(done),'failures':sum(not r['ok'] for r in done)}),flush=True)
   time.sleep(10)
  for j in jobs:j.result()
 final=get('status','final');emit({'kind':'measurement_end','final':final})
 selected=[r for r in records if r.get('client') in ('capture1','capture2','status')]
 summary={'label':a.label,'duration':a.duration,'requests':len(selected),'success':sum(r['ok'] for r in selected),'failures':sum(not r['ok'] for r in selected),'captures':sum(r['kind']=='capture' for r in selected),'capture_success':sum(r['kind']=='capture' and r['ok'] for r in selected)}
 (out/(a.label+'-summary.json')).write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
finally:
 stop.set();t.join(2);s.close();uart.close();log.close()
