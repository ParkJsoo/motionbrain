import collections,json,pathlib,statistics
root=pathlib.Path(__file__).resolve().parent
all_results={}
for name in ('current-10m','before-10m','current-live-5m'):
 path=root/(name+'.jsonl')
 if not path.exists():continue
 records=[json.loads(line) for line in path.read_text().splitlines()]
 selected=[r for r in records if r.get('client') in ('capture1','capture2','status')]
 status=[r['status'] for r in selected if r['kind']=='status' and r.get('ok')]
 capture=[r for r in selected if r['kind']=='capture']
 good=[r for r in capture if r['ok']]
 frames=collections.defaultdict(set)
 for r in good:frames[r.get('sourceFrameId')].add(r['sha256'])
 baseline=next((r['baseline'].get('status') for r in records if r['kind']=='measurement_start'),None)
 final=next((r['final'].get('status') for r in records if r['kind']=='measurement_end'),None)
 uart=(root/(name+'-uart.txt')).read_text()
 durations=sorted(r['seconds'] for r in good)
 all_results[name]={'complete':any(r['kind']=='measurement_end' for r in records),'requests':len(selected),'errors':sum(not r['ok'] for r in selected),'capture_attempts':len(capture),'capture_success':len(good),'status_attempts':sum(r['kind']=='status' for r in selected),'status_success':len(status),'missed_scheduled_slots':sum(r['count'] for r in records if r['kind']=='missed_slots'),'uart_reset_reasons':[line for line in uart.splitlines() if 'Reset reason:' in line],'uart_panic_count':uart.count('Guru Meditation'),'boot_ids':sorted(set(r['cameraBootId'] for r in status if 'cameraBootId' in r)),'uptime_regressions':sum(b['uptimeMs']<a['uptimeMs'] for a,b in zip(status,status[1:])),'rssi_range':[min(r['rssi'] for r in status),max(r['rssi'] for r in status)] if status else None,'heap_range':[min(r['heapFree'] for r in status),max(r['heapFree'] for r in status)] if status else None,'capture_latency_seconds':{'median':statistics.median(durations),'max':max(durations)} if durations else None,'metadata_missing':sum(r.get('sourceFrameId') is None for r in good),'conflicting_frame_hashes':sum(len(v)>1 for k,v in frames.items() if k is not None),'initial':baseline,'final':final,'camera_capture_counter_delta': final['captureRequests']-baseline['captureRequests'] if baseline and final and final['uptimeMs']>baseline['uptimeMs'] else None}
(root/'analysis.json').write_text(json.dumps(all_results,indent=2)+'\n')
for name,r in all_results.items():print(name,json.dumps({k:v for k,v in r.items() if k not in ('initial','final')}))
