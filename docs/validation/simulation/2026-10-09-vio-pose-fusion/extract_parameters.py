"""Read P parameter records from a ULog, using locked PX4 messages.h layout."""
import argparse,hashlib,json,struct
from pathlib import Path


def extract(path,expected):
    data=path.read_bytes()
    if data[:7]!=b'ULog\x01\x12\x35':raise ValueError('Invalid ULog magic')
    offset=16;values={};histories={};prefix_end=None
    while offset+3<=len(data):
        size,kind=struct.unpack_from('<HB',data,offset)
        payload=data[offset+3:offset+3+size]
        if len(payload)!=size:raise ValueError('Truncated ULog message')
        if kind==ord('D') and prefix_end is None:prefix_end=offset
        if kind==ord('P'):
            n=payload[0];dtype,name=payload[1:1+n].decode().split(' ',1)
            if name in expected:
                value=struct.unpack('<i' if dtype=='int32_t' else '<f',payload[1+n:])[0]
                values[name]=value;histories.setdefault(name,[]).append(dict(offset=offset,value=value))
        offset+=3+size
    if offset!=len(data):raise ValueError('Truncated ULog record header')
    checks={name:name in histories and all(r['value']==target for r in histories[name]) for name,target in expected.items()}
    return dict(passed=all(checks.values()),checks=checks,parameters=values,history=histories,
        source=str(path),source_sha256=hashlib.sha256(data).hexdigest(),
        initial_header_bytes=prefix_end,scope='actual ULog parameter records; no camera or flight claim'),data[:prefix_end]


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('ulog',type=Path);parser.add_argument('manifest',type=Path)
    parser.add_argument('output',type=Path);args=parser.parse_args()
    manifest=json.loads(args.manifest.read_text())
    expected={name.removeprefix('PX4_PARAM_'):float(value) for name,value in manifest['px4_parameter_overrides'].items()}
    result,header=extract(args.ulog,expected)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    args.output.with_suffix('.ulog-header').write_bytes(header)
    print('ULog parameters:',result['passed'],result['parameters'])
    raise SystemExit(0 if result['passed'] else 1)
