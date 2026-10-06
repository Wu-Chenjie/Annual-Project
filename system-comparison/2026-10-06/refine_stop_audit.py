"""Read-only handling of a pending lease held through a task-end-censored stop.

Never count an unexecuted authorization as a consumed handoff. Preserve the
original audit, and change only its missing-consumption error when a newer
cancel was executed before the boundary and both leases remain held at the
end of the recording. Every other original error remains an error.
"""
import json,math,sys
from pathlib import Path

def records(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]

def proof(handoff,events,commands,executions,states,end):
    old,new=handoff['old_token'],handoff['token']
    authorized=handoff['authorized_at']
    proposals=[e for e in events if e['type']=='handoff_proposed' and e.get('token')==new]
    if len(proposals)!=1:return None
    proposal=proposals[0];drone=proposal['drone'];boundary=proposal['boundary']['trajectory_time']
    for stop in commands:
        t=stop.get('receipt_time');epoch=stop.get('epoch')
        if not (stop.get('cancel') and stop.get('drone')==drone and t is not None and authorized<=t<end and epoch is not None):continue
        matching=[e for e in events if e['type']=='lease_cancellation_requested' and e['drone']==drone and e.get('reason')==stop.get('reason') and abs(e['time']-t)<=.25]
        if not matching:continue
        before=[e for e in executions if e['drone']==drone and e.get('token')==old and 0<=t-e['time']<=.25]
        after=[e for e in executions if e['drone']==drone and t<=e['time']<=end]
        held=[s for s in states if s['drone']==drone and t+.2<=s['time']<=end]
        if not before or len(after)<2 or not held:continue
        last=max(before,key=lambda e:e['time'])
        if last['trajectory_time']+t-last['time']>=boundary-.05:continue
        if epoch<=last['epoch']:continue
        times=[e['time'] for e in after]
        if min(times)>t+.25 or max(times)<end-.25 or any(b-a>.6 for a,b in zip(sorted(times),sorted(times)[1:])):continue
        if any(e.get('token') is not None or e.get('pending_token') is not None or e['epoch']!=epoch or
               math.sqrt(sum(v*v for v in e.get('reference_velocity',[float('inf')]*3)))>1e-8 for e in after):continue
        if max(s['time'] for s in held)<end-.6:continue
        for state in held:
            intents=[state.get('intent'),state.get('pending_intent')]+state.get('retiring_intents',[])
            tokens={p['token'] for p in intents if p and p.get('retiring') and p.get('committed')}
            if not {old,new}<=tokens:break
        else:return dict(token=new,old_token=old,executed_stop_at=t,stop_epoch=epoch,
            last_old_progress=last['trajectory_time'],boundary=boundary,held_until=max(s['time'] for s in held),
            task_end_censored_at=end,consumed=False,classification='cancelled_before_boundary_with_leases_retained')
    return None


def refine(root):
    root=Path(root);report=json.loads((root/'native-protocol-audit.json').read_text())
    end=json.loads((root/'result.json').read_text())['t']
    events=[e for f in root.glob('drone_*/events.jsonl') for e in records(f)]
    commands=records(root/'command-evidence.jsonl');executions=records(root/'execution-evidence.jsonl');states=records(root/'peer_states.jsonl')
    proven=[]
    for handoff in report['handoffs']:
        error='Authorized handoff not consumed '+handoff['token']
        if error not in report['errors']:continue
        found=proof(handoff,events,commands,executions,states,end)
        if found:report['errors'].remove(error);proven.append(found)
    report.update(passed=not report['errors'],status='FAIL' if report['errors'] else ('PASS_WITH_STOP_PROOF' if proven else 'PASS'),read_only_stop_proofs=proven,
        original_audit='native-protocol-audit.json',note='Only retained-lease task-end-censored stops are recognized; no consumption or lease retirement is fabricated.')
    (root/'native-protocol-stop-reviewed.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':
    r=refine(sys.argv[1]);print(json.dumps(dict(passed=r['passed'],errors=r['errors'],proofs=r['read_only_stop_proofs']),indent=2))
