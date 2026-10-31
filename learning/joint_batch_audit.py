"""Offline HTTP/output evidence audit, including proven pre-dispatch cancellation."""
import json


def read(path):return json.loads(path.read_text())


def validate_cancel_evidence(attempt,error,trace):
    """A guard rejection must have known zero output through completed cleanup."""
    if (error.get('error')!='ValueError' or attempt['dispatchGuardPassed'] is not False
            or attempt['executionStatus']!='not_dispatched' or attempt['offVerified'] is not True):
        raise ValueError('unproven_predispatch_cancel')
    if (not trace or any(e['pwm']!=0 for e in trace)
            or {e['motor'] for e in trace}!={1,2,3,4,5}):raise ValueError('cancel_output_not_proven_off')


def audit_commands(root,plan,report,remember,t):
    initial=json.loads(remember(root/'campaign-initial/initial-state.json'))
    boot=initial['outputs']['bootId'];control=dict(controllerBootId=boot,expectedBoundedRecordCount=0)
    checker=t.Port(plan['controllerUrl'],'unused-offline',None)
    t.check_snapshot(initial,control,checker)
    output={};exchanges=[];written=False;retry=0;off_cache={}
    attempts=report['commandAttempts']
    if attempts!=[a for row in report['trials'] for a in row['commandAttempts']]:raise ValueError('attempt_records_differ')
    run_intents=[];aborted=[]
    paths=sorted(root.glob('http-*-intent.json'))
    for pos,path in enumerate(paths):
        intent=json.loads(remember(path));response=path.with_name(path.name.replace('-intent','-response'))
        is_run=intent['method']=='POST' and intent['path']=='/sequence' and intent['payload']=={'action':'run'}
        if is_run:run_intents.append(intent)
        if not response.exists():
            error=json.loads(remember(path.with_name(path.name.replace('-intent','-error'))))
            if is_run:
                index=len(run_intents)-1
                if index>=len(attempts):raise ValueError('unaccounted_run_intent')
                a=attempts[index];label=a['label']
                if (error.get('error')!='ValueError' or a['dispatchGuardPassed'] is not False
                        or a['executionStatus']!='not_dispatched' or a['offVerified'] is not True
                        or (root/(a['trialId']+'-dispatch.json')).exists()
                        or (root/label/'execution-receipt.json').exists()):raise ValueError('unproven_predispatch_cancel')
                before=json.loads(remember(root/label/'pre-execution-state.json'))
                final=json.loads(remember(root/label/'final-state.json'))
                cancel_checker=t.Port(plan['controllerUrl'],'unused-offline',None)
                t.check_snapshot(before,control,cancel_checker);t.check_snapshot(final,control,cancel_checker)
                if before['sequence']['sequence']['totalCount']!=0 or final['sequence']['sequence']['totalCount']!=0:raise ValueError('cancel_queue_not_empty')
                base=before['outputs']['entries'][-1]['sequence']
                trace=[e for e in final['outputs']['entries'] if e['sequence']>base]
                validate_cancel_evidence(a,error,trace)
                following=[read(q) for q in paths[pos+1:]]
                writes=[q for q in following if q['method']=='POST']
                if len(writes)<2 or writes[0]['path']!='/command' or writes[0]['payload']!={'cmd':'stop'} or writes[1]['path']!='/sequence' or writes[1]['payload']!={'action':'clear'}:raise ValueError('cancel_stop_clear_missing')
                aborted.append(index);continue
            if intent['method']!='GET' or error.get('error') not in ('TimeoutError','URLError') or set(off_cache)!={'/actions','/status','/action_outputs'}:
                raise ValueError('http_unknown_outcome_not_auditable')
            t.check_off_prefix(dict(capability=off_cache['/actions'],status=off_cache['/status'],outputs=off_cache['/action_outputs']),control,t.Port(plan['controllerUrl'],'unused',None))
            retry+=1;r=json.loads(remember(root/f'idle-retry-{retry:03d}.json'))
            if r['path']!=intent['path'] or r['failedAttempt']!=1 or r['nextAttempt']!=2 or r['outputPreviouslyVerifiedOff'] is not True or r['delaySeconds']!=.1:raise ValueError('invalid_idle_retry')
            if pos+1>=len(paths):raise ValueError('missing_retry')
            nxt=read(paths[pos+1])
            if nxt['method']!='GET' or nxt['path']!=intent['path'] or nxt['payload'] is not None or nxt['startedAt']<r['at']+.09 or not paths[pos+1].with_name(paths[pos+1].name.replace('-intent','-response')).exists():raise ValueError('invalid_retry_successor')
            continue
        response=json.loads(remember(response))
        if t.response_json(response['rawUtf8'],intent['path'])!=response['parsed'] or response['httpStatus']!=200:raise ValueError('http_response_mismatch')
        wait=0.
        if is_run:
            index=len(run_intents)-1;wf=root/f'dispatch-wait-{index+1:03d}.json'
            if wf.exists():
                w=json.loads(remember(wf));wait=w['endedAt']-w['startedAt']
                if (w['maxSeconds']!=3 or not 0<=wait<=3 or type(w['checks']) is not int or w['checks']<1
                        or not intent['startedAt']<=w['startedAt']<=w['endedAt']<=response['receivedAt']):raise ValueError('invalid_dispatch_wait')
        limit=2.1 if not written and intent['method']=='GET' else .6
        if not 0<=response['receivedAt']-intent['startedAt']<=(1.2+wait if is_run else limit):raise ValueError('http_deadline')
        if intent['method']=='POST':
            written=True
            if response['parsed'].get('success') is not True:raise ValueError('post_failed')
        if intent['method']=='POST' and (intent['path']=='/command' or intent['payload']=={'action':'run'}):off_cache={}
        elif intent['method']=='GET' and intent['path'] in ('/actions','/status','/action_outputs'):off_cache[intent['path']]=response['parsed']
        if intent['path']=='/action_outputs':
            out=response['parsed']
            if out['bootId']!=boot or out['evidence']!='driver_write':raise ValueError('output_identity_changed')
            for e in out['entries']:
                if e['sequence'] in output and output[e['sequence']]!=e:raise ValueError('output_history_changed')
                output[e['sequence']]=e
        exchanges.append((intent,response))
    if retry!=len(list(root.glob('idle-retry-*.json'))):raise ValueError('unmatched_retry')
    ordered=sorted(output)
    if any(b!=a+1 for a,b in zip(ordered,ordered[1:])):raise ValueError('output_history_gap')
    runs=[(n,a,b) for n,(a,b) in enumerate(exchanges) if a['path']=='/sequence' and a['payload']=={'action':'run'}]
    adds=[a['payload'] for a,b in exchanges if a['path']=='/sequence' and a['payload'] and a['payload'].get('action')=='add']
    executed=[r for r in report['trials'] if r['executed'] is True]
    if len(run_intents)!=len(attempts) or len(adds)!=len(attempts) or len(runs)+len(aborted)!=len(attempts) or len(runs)!=len(executed) or any(r['executed'] is None for r in report['trials']):raise ValueError('command_count_or_outcome_unknown')
    for i,a in enumerate(attempts):
        saved=json.loads(remember(root/(a['label']+'-intent.json')))
        expected=dict(action='add',joint='wrist',direction=a['pulse']['direction'],speed=63,duration=a['pulse']['durationMs'])
        if saved['pulse']!=a['pulse'] or saved['trial']['id']!=a['trialId'] or adds[i]!=expected:raise ValueError('attempt_add_changed')
        if i not in aborted and (a['executionStatus']!='executed' or a['dispatchGuardPassed'] is not True or a['offVerified'] is not True):raise ValueError('unknown_dispatch_attempt')
    adds=[a for i,a in enumerate(adds) if i not in aborted]
    accounted=set()
    for row,add,(n,intent,response) in zip(executed,adds,runs):
        pulse=row['pulse'];receipt=row['execution'];label=row['commandAttempts'][-1]['label']
        stored=json.loads(remember(root/label/'execution-receipt.json'))
        if stored!=receipt or add!=dict(action='add',joint='wrist',direction=pulse['direction'],speed=63,duration=pulse['durationMs']):raise ValueError('executed_command_changed')
        dispatch=json.loads(remember(root/(row['trial']['id']+'-dispatch.json')))
        if dispatch!=dict(at=row['dispatchAt'],observation=row['observation'],pulse=pulse):raise ValueError('dispatch_changed')
        if not intent['startedAt']<=receipt['dispatchAt']<=response['receivedAt'] or not 0<=response['receivedAt']-receipt['dispatchAt']<=.75 or not receipt['dispatchAt']<=receipt['offAt']<receipt['dispatchAt']+3.75:raise ValueError('command_clock_mismatch')
        off=json.loads(remember(root/label/'output-off-state.json'));t.check_snapshot(off,control,checker)
        preceding=sorted([(b['receivedAt'],b['parsed']) for a,b in exchanges if a['path']=='/action_outputs' and b['receivedAt']<=receipt['dispatchAt']],key=lambda x:x[0])
        previous=preceding[-1][1]
        baseline=previous['entries'][-1]['sequence'];trace=[e for e in off['outputs']['entries'] if e['sequence']>baseline]
        if trace!=receipt['outputTrace'] or receipt['offVerified'] is not True:raise ValueError('output_trace_mismatch')
        sign=1 if pulse['direction']=='up' else -1
        if not any(e['motor']==2 and e['pwm']==0 for e in trace) or not any(e['pwm']!=0 for e in trace):raise ValueError('missing_pulse_or_stop')
        for e in trace:
            if output.get(e['sequence'])!=e or (e['pwm'] and (e['motor']!=2 or not 0<sign*e['pwm']<=160)):raise ValueError('wrong_motor_output')
            accounted.add(e['sequence'])
    baseline=initial['outputs']['entries'][-1]['sequence']
    if any(e['sequence']>baseline and e['pwm']!=0 and e['sequence'] not in accounted for e in output.values()):raise ValueError('unexpected_quiet_output')
    final=json.loads(remember(root/'campaign-final/final-state.json'));t.check_snapshot(final,control,checker)
    if final['sequence']['sequence']['totalCount']!=0:raise ValueError('final_queue_not_empty')
