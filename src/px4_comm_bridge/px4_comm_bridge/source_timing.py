"""Bounded, passive timing evidence. Diagnostics never change source admission."""
import json
import time
from collections import deque
from pathlib import Path


class SourceTiming:
    def __init__(self,limit=20000):
        self.records=deque(maxlen=limit)
        self.total=0

    def record(self,stage,ros,sample=None,info=None,**details):
        metadata={}
        if isinstance(info,dict):
            # rclpy exposes host/RMW timestamps and sequence numbers, not GID.
            metadata={k:int(v) for k,v in info.items() if isinstance(v,int) and not isinstance(v,bool)}
        self.records.append(dict(stage=stage,mono=time.monotonic(),system_ns=time.time_ns(),
            ros=ros,sample=sample,age_s=ros-sample if sample is not None else None,
            rmw=metadata,**details))
        self.total+=1

    def write(self,path):
        Path(path).write_text(json.dumps(dict(schema=1,total=self.total,retained=len(self.records),
            scope='passive callback timing; host timestamps require stable shared host clock',
            records=list(self.records)),indent=2)+'\n')
