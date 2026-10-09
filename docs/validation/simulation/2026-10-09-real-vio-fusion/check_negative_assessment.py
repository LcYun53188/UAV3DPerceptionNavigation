"""Mutate in-memory copies only; archived flight/source evidence remains untouched."""
import copy,importlib.util,json
from pathlib import Path

base=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('assessor',base.parents[3]/'scripts/assess_px4_real_vio_fusion.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
folder=base/'actual-pose-paced'
names=('result.json','manifest.json','fusion-outputs.json','fusion-dds-echoes.json',
       'fusion-telemetry.json','normalized-poses.json','sdk-poses.json','normalized-status.json')
payloads={n:module.read(folder,n) for n in names}
module.read=lambda _,name:payloads[name]
assert module.assess(folder)['passed']


def fake_velocity(d):d[0]['message']['velocity'][0]=0.
def stale_sample(d):d[0]['ev']['timestamp_sample']-=300000
def clamp_covariance(d):d[0]['aligned']['pose']['covariance'][0]*=.5
def wrong_coordinate(d):d[0]['ev']['position'][0]+=1.
def wrong_source(d):d[100]['position'][0]+=1.
def hidden_gnss(d):d['real_pose_fusion']['before_stop']['flags']['cs_gnss_pos']=True
def late_fusion(d):d['real_pose_fusion']['last_fuse_at_end']['ev_pos']+=1
def missing_fusion(d):
    for e in d:
        if e['name']=='ev_pos':e['message']['fused']=False


cases=(('fake_velocity','fusion-dds-echoes.json',fake_velocity),
       ('stale_sample','fusion-outputs.json',stale_sample),
       ('clamped_covariance','fusion-outputs.json',clamp_covariance),
       ('wrong_coordinate','fusion-outputs.json',wrong_coordinate),
       ('wrong_source','normalized-poses.json',wrong_source),
       ('hidden_gnss','result.json',hidden_gnss),
       ('continued_fusion','result.json',late_fusion),
       ('missing_actual_fusion','fusion-telemetry.json',missing_fusion))
results={}
for name,file,mutate in cases:
    original=payloads[file];payloads[file]=copy.deepcopy(original);mutate(payloads[file])
    result=module.assess(folder)
    results[name]=dict(rejected=not result['passed'],failed_checks=[n for n,v in result['checks'].items() if not v])
    payloads[file]=original
print(json.dumps(dict(passed=all(r['rejected'] for r in results.values()),cases=results),indent=2))
raise SystemExit(0 if all(r['rejected'] for r in results.values()) else 1)
