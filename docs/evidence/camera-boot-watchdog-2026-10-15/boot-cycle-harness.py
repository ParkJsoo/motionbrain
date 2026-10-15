import argparse,json,pathlib,threading,time,urllib.request,serial
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--cycles',type=int,default=10);a=p.parse_args()
out=pathlib.Path('/tmp/motionbrain-cam-boot');out.mkdir(exist_ok=True)
f=(out/(a.label+'-uart.txt')).open('w');lock=threading.Lock();stop=threading.Event();epoch=time.monotonic();lines=[];results=[]
s=serial.Serial();s.port='/dev/cu.usbserial-1120';s.baudrate=115200;s.timeout=.1;s.dtr=False;s.rts=False;s.open();s.reset_input_buffer()
def record(line):
 with lock:lines.append((time.monotonic()-epoch,line));f.write(f'{lines[-1][0]:.3f}s {line}\n');f.flush()
def read():
 while not stop.is_set():
  line=s.readline().decode('utf-8',errors='replace').strip()
  if line and not any(k in line.lower() for k in ('ssid','password','token')):record(line)
t=threading.Thread(target=read);t.start()
try:
 for i in range(a.cycles):
  start=time.monotonic();begin=start-epoch
  record(f'HOST_CAMERA_RESET cycle={i+1}')
  s.rts=True;time.sleep(.1);s.rts=False
  while time.monotonic()-start<45:
   with lock:ready=[(ts,l) for ts,l in lines if ts>=begin and 'Camera HTTP server ready' in l]
   if ready and time.monotonic()-start>=20:break
   time.sleep(.1)
  result={'cycle':i+1,'window_seconds':round(time.monotonic()-start,3),'ready_times':[round(ts-begin,3) for ts,l in ready]}
  with lock:result['reset_reasons']=[l for ts,l in lines if ts>=begin and 'Reset reason:' in l];result['panic_count']=sum('Guru Meditation' in l for ts,l in lines if ts>=begin)
  try:
   with urllib.request.urlopen('http://192.168.219.116/status',timeout=4) as response:d=json.load(response)
   result['status']={k:d.get(k) for k in ('cameraBootId','uptimeMs','resetReason','rssi','heapFree','captureFailures')}
   with urllib.request.urlopen('http://192.168.219.116/capture',timeout=4) as response:
    raw=response.read();result['capture_ok']=response.status==200 and raw.startswith(b'\xff\xd8') and raw.endswith(b'\xff\xd9')
  except Exception as e:result['error']=str(e)
  results.append(result);(out/(a.label+'-results.json')).write_text(json.dumps(results,indent=2));print(json.dumps(result),flush=True)
  if not ready:raise RuntimeError('No HTTP server ready in 45 seconds; stop resets for investigation')
finally:stop.set();t.join(2);s.close();f.close()
