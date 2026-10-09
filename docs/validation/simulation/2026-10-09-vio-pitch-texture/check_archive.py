#!/usr/bin/env python3
"""Check bytes, source assessments and independent motion replay; no flight claim."""
import hashlib
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[3]/'scripts'))
from assess_vio_warehouse import assess
from assess_vio_motion_evidence import replay


def main():
    for line in (ROOT/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise ValueError('Hash mismatch: '+name)
    for run,expected in [('43fe5128-f399-41dc-9687-a47bc2d7a4b4',True),('36ba47bb-7274-46b9-816f-81d0b883c2a1',True),('327fc31f-b971-4b19-8036-fead92c54b86',False)]:
        r=assess(ROOT/run)
        if r['passed']!=expected:raise ValueError('Unexpected assessment: '+run)
        if expected:
            q=replay(ROOT/run)
            if not all(q[k] for k in ('error_limits_met','reported_metrics_agree','original_overall_passed','cleanup_confirmed','fmu_inputs_absent')):raise ValueError('Independent motion replay failed: '+run)
    print('Archive bytes and source outcomes verified; no flight qualification.')

if __name__=='__main__':main()
