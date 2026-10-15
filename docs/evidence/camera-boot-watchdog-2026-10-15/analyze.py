import json,pathlib,re
root=pathlib.Path(__file__).resolve().parent
result={}
for label,count in [('baseline',10),('diag',10),('fixed-diag',5),('production-fixed',5)]:
 path=root/(label+'-results.json')
 if not path.exists():continue
 cycles=json.loads(path.read_text());uart=(root/(label+'-uart.txt')).read_text()
 stages=[dict(re.findall(r'(\w+)=([\w.-]+)',line)) for line in uart.splitlines() if 'BOOT_STAGE' in line]
 names=[dict(re.findall(r'(\w+)=([\w.-]+)',line)) for line in uart.splitlines() if 'WIFI_NETIF_HOSTNAME' in line]
 sockets=[dict(re.findall(r'(\w+)=([\w.-]+)',line)) for line in uart.splitlines() if 'SOCKET_TIMEOUT' in line]
 result[label]={'complete':len(cycles)==count,'uart_boot_count':uart.count('Reset reason:'),'uart_panic_total':uart.count('Guru Meditation'),'cycles':len(cycles),'server_ready_cycles':sum(len(x['ready_times'])==1 for x in cycles),'unexpected_restarts':sum(len(x['reset_reasons'])!=1 or x['reset_reasons'][0]!='Reset reason: poweron' for x in cycles),'panic_count':sum(x['panic_count'] for x in cycles),'capture_success':sum(x.get('capture_ok',False) for x in cycles),'request_errors':[{'cycle':x['cycle'],'error':x['error']} for x in cycles if 'error' in x],'ready_seconds_range':[min(x['ready_times'][0] for x in cycles if x['ready_times']),max(x['ready_times'][0] for x in cycles if x['ready_times'])],'heap_integrity_failures':sum(x.get('integrity')!='1' for x in stages),'stack_min':min(int(x['stack']) for x in stages) if stages else None,'actual_hostnames':sorted(set(x['actual'] for x in names)),'socket_samples':len(sockets),'socket_timeout_failures':sum(any(x.get(k)!=v for k,v in {'configured':'1','stream_ms':'750','rcv_result':'0','rcv_s':'0','rcv_us':'750000','snd_result':'0','snd_s':'0','snd_us':'750000'}.items()) for x in sockets)}
(root/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
for label,data in result.items():print(label,json.dumps(data))
