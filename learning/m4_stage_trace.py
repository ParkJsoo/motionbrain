"""Audit bounded firmware RAM observations; never convert them to wire time."""
import hashlib
import re

from learning.artifacts import _decode_json
from learning.mac_clock_bridge import finite

SCHEMA='motionbrain.m4_stage_trace.v1'
STAGES=('requestParsedUs','sensorUpdateEnteredUs','afterAdcUs','pollStartedUs',
        'angleReadStartedUs','angleReadCompletedUs','pollCompletedUs','replyEnqueueUs','replyWriteReturnedUs')
LOOP_FIELDS=('requestLoopStartedUs','requestLoopGapUs','replyLoopStartedUs','maxPendingLoopGapUs')


def integer(value, limit=2**64):
    return type(value) is int and 0<=value<limit


def check_diagnostics(receipt):
    """Validate GET-only receipt and firmware schema before correlating nonces."""
    r=receipt
    if (not isinstance(r,dict) or r.get('endpoint')!='m4_diagnostics' or r.get('method')!='GET'
            or r.get('transport')!='python_http_client' or r.get('transportCompleted') is not True
            or r.get('httpStatus')!='200' or r.get('clockDomain')!='python_monotonic'):
        raise ValueError('firmware_diagnostic_transport_invalid')
    a,b=r.get('requestStartedMonotonic'),r.get('receivedMonotonic')
    if (not all(finite(v) for v in (a,b,r.get('elapsedMs'))) or not 0<=a<=b
            or not 0<=r['elapsedMs']<=800 or abs((b-a)*1000-r['elapsedMs'])>.001):
        raise ValueError('firmware_diagnostic_receipt_clock_invalid')
    body=r.get('responseBody')
    if (not isinstance(body,str) or hashlib.sha256(body.encode()).hexdigest()!=r.get('bodySha256')
            or _decode_json(body.encode())!=r.get('response')):raise ValueError('firmware_diagnostic_body_invalid')
    return validate_snapshot(r['response'])


def validate_snapshot(value):
    if (not isinstance(value,dict) or value.get('schemaVersion')!=SCHEMA
            or value.get('clockDomain')!='esp_timer_since_boot_us' or value.get('capacity')!=8
            or any(value.get(k) is not False for k in ('clockRateCalibrated','physicalSampleTimeVerified','txCompleteVerified'))
            or not isinstance(value.get('bootId'),str) or not re.fullmatch('[0-9a-f]{32}',value['bootId'])
            or not integer(value.get('nextSequence')) or value['nextSequence']<1
            or not integer(value.get('dropped')) or type(value.get('active')) is not bool):
        raise ValueError('firmware_diagnostic_contract_invalid')
    records=value.get('records')
    if not isinstance(records,list) or len(records)>8:raise ValueError('firmware_diagnostic_records_invalid')
    if value['nextSequence']!=value['dropped']+len(records)+1:
        raise ValueError('firmware_diagnostic_sequence_accounting')
    for i,row in enumerate(records):
        if (not isinstance(row,dict) or not integer(row.get('sequence')) or row['sequence']!=value['dropped']+i+1
                or not isinstance(row.get('nonce'),str) or not re.fullmatch('[0-9a-f]{8}',row['nonce'])
                or type(row.get('protocol')) is not int or row['protocol'] not in (2,3)
                or row.get('status') not in ('pending','replied','cancelled','expired','format_failed','write_failed')):
            raise ValueError('firmware_diagnostic_record_invalid')
        for key in STAGES+LOOP_FIELDS:
            if key not in row or row[key] is not None and not integer(row[key]):raise ValueError('firmware_stage_time_invalid')
        for key in ('requestReceivedMs','pollMarkerMs','replyReportedMs','replyPollMarkerMs','pollAttempts'):
            if key not in row or row[key] is not None and not integer(row[key],2**32):raise ValueError('firmware_stage_marker_invalid')
        for key in ('replyBytes','writtenBytes','replyRaw'):
            if key not in row or row[key] is not None and not integer(row[key],4096 if key=='replyRaw' else 91):
                raise ValueError('firmware_stage_value_invalid')
        for key in ('pollSucceeded','angleSucceeded','adcSampled'):
            if key not in row or row[key] is not None and type(row[key]) is not bool:raise ValueError('firmware_stage_boolean_invalid')
    pending=[row for row in records if row['status']=='pending']
    if len(pending)>1 or value['active']!=bool(pending) or pending and pending[0] is not records[-1]:
        raise ValueError('firmware_pending_state_invalid')
    return value


def link_stages(before,after,samples):
    """Six current S3 records; ring overwrite of older records is explicit."""
    validate_snapshot(before);validate_snapshot(after)
    if (before['active'] or after['active'] or before['bootId']!=after['bootId'] or len(samples)!=6
            or after['nextSequence']!=before['nextSequence']+6
            or after['dropped']!=before['dropped']+max(0,len(before['records'])+6-8)):
        raise ValueError('firmware_stage_session_changed')
    current=[row for row in after['records'] if row['sequence']>=before['nextSequence']]
    if len(current)!=6:raise ValueError('firmware_stage_current_records_missing')
    retained=8-len(current)
    expected_prior=before['records'][-retained:] if retained else []
    if after['records'][:-6]!=expected_prior:
        raise ValueError('firmware_stage_previous_records_changed')
    rows=[]
    for row,sample in zip(current,samples):
        if (row['status']!='replied' or row['protocol']!=3 or row['nonce']!=sample['nonce']
                or after['bootId']!=sample['bootId'] or row['replyBytes']!=58 or row['writtenBytes']!=58
                or row['pollSucceeded'] is not True or row['angleSucceeded'] is not True or not integer(row['pollAttempts'],2**32) or row['pollAttempts']<1
                or row['pollMarkerMs']!=sample['polledMs'] or row['replyPollMarkerMs']!=sample['polledMs']
                or row['replyReportedMs']!=sample['reportedMs'] or row['requestReceivedMs']!=sample['requestReceivedMs']
                or row['replyRaw']!=sample['raw']):raise ValueError('firmware_stage_sample_mismatch')
        times=[row[k] for k in STAGES]
        if (any(v is None for v in times) or times!=sorted(times)
                or any(row[k] is None for k in LOOP_FIELDS)
                or not row['requestLoopStartedUs']<=row['requestParsedUs']
                or not row['requestLoopStartedUs']<=row['replyLoopStartedUs']<=row['sensorUpdateEnteredUs']
                or times[-1]-times[0]>800000):raise ValueError('firmware_stage_order_invalid')
        intervals={f'{a}To{b}':(row[b]-row[a])/1000 for a,b in zip(STAGES,STAGES[1:])}
        rows.append(dict(sequence=row['sequence'],nonce=row['nonce'],observedStageIntervalsMs=intervals,
                         pollAttempts=row['pollAttempts'],latestPollOnly=True,adcSampled=row['adcSampled'],
                         requestLoopGapMs=row['requestLoopGapUs']/1000,
                         maxPendingLoopGapMs=row['maxPendingLoopGapUs']/1000,
                         parseToWriteReturnMs=(times[-1]-times[0])/1000))
    return dict(schemaVersion='motionbrain.m4_stage_link.v1',bootId=after['bootId'],records=rows,
                overwrittenPriorRecords=after['dropped']-before['dropped'],clockRateCalibrated=False,
                physicalSampleTimeVerified=False,txCompleteVerified=False,trainingEligible=False,controlEligible=False)
