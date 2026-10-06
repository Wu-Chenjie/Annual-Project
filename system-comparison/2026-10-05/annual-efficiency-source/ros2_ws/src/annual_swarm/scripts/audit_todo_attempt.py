#!/usr/bin/env python3
"""One common frozen audit implementation for both policies and all outcomes."""
import argparse
import json
from pathlib import Path

from audit_todo_evidence import audit as accounting
from verify_todo_run import verify
from audit_execution_protocol import audit as protocol


def audit(root):
    root=Path(root);result={}
    for name,operation in [('accounting',accounting),('flight_and_coverage',verify),('execution_protocol',protocol)]:
        try:
            data=operation(root)
            result[name]=dict(completed=True,passed=data.get('passed'),status=data.get('status'))
        except Exception as exc:result[name]=dict(completed=False,error=repr(exc))
    outcome=json.loads((root/'run-result.json').read_text())['outcome']
    result['outcome']=outcome
    result['complete_attempt_accepted']=(outcome=='COMPLETE' and result['accounting']['completed'] and
        result['flight_and_coverage'].get('passed') is True and result['execution_protocol'].get('passed') is True
        and result['execution_protocol'].get('status')=='PASS')
    (root/'audit-status.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    print(json.dumps(audit(args.directory),indent=2))
