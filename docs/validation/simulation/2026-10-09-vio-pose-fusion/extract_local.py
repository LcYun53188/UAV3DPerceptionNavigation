"""Capture a ULog local-position record at or immediately before stop snapshot."""
import argparse,hashlib,json,re,struct
from pathlib import Path


def decoder(description):
    types={'uint64_t':'Q','double':'d','float':'f','bool':'?','uint8_t':'B','int8_t':'b',
        'uint32_t':'I','int32_t':'i','uint16_t':'H','int16_t':'h'}
    fields=[];fmt='<'
    for field in description.split(':',1)[1].split(';'):
        if not field:continue
        dtype,name=field.split();match=re.fullmatch(r'(\w+)(?:\[(\d+)\])?',dtype)
        count=int(match[2] or 1);fmt+=types[match[1]]*count;fields.append((name,count))
    def decode(payload):
        raw=iter(struct.unpack(fmt,payload));result={}
        for name,count in fields:result[name]=next(raw) if count==1 else [next(raw) for _ in range(count)]
        return result
    return decode


def capture(ulog,target,output):
    data=ulog.read_bytes();offset=16;fmt=None;ident=None;best=None
    while offset+3<=len(data):
        n,kind=struct.unpack_from('<HB',data,offset);body=data[offset+3:offset+3+n]
        if len(body)!=n:raise ValueError('Truncated ULog')
        if kind==70 and body.startswith(b'vehicle_local_position:'):fmt=body.decode()
        if kind==65 and body[3:]==b'vehicle_local_position' and body[0]==0:ident=struct.unpack_from('<H',body,1)[0]
        if kind==68 and ident is not None and struct.unpack_from('<H',body)[0]==ident:
            decoded=decoder(fmt)(body[2:])
            if decoded['timestamp']<=target and (best is None or decoded['timestamp']>best[0]['timestamp']):
                best=decoded,offset,data[offset:offset+3+n]
        offset+=3+n
    if best is None or target-best[0]['timestamp']>50000:raise ValueError('Missing fresh ULog sample before stop')
    record,offset,raw=best
    selected={n:record[n] for n in ('timestamp','timestamp_sample','x','y','z','vx','vy','vz','eph','epv','evh','evv',
        'xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control','heading_var','dead_reckoning')}
    output.write_text(json.dumps(selected,indent=2)+'\n')
    output.with_suffix('.bin').write_bytes(raw)
    output.with_suffix('.format.json').write_text(json.dumps(dict(format=fmt,target_timestamp=target,
        source=str(ulog),source_sha256=hashlib.sha256(data).hexdigest(),offset=offset,
        raw_sha256=hashlib.sha256(raw).hexdigest(),scope='single original ULog record; not a complete flight log'),indent=2)+'\n')
    print('Captured local ULog:',selected)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('ulog',type=Path);parser.add_argument('result',type=Path)
    parser.add_argument('output',type=Path);args=parser.parse_args()
    capture(args.ulog,json.loads(args.result.read_text())['local_before_stop']['timestamp'],args.output)
