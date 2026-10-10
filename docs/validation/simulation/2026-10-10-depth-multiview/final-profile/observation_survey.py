"""Finite survey inside a frozen known region; never a map-cleared navigation path."""
import math
from .flight_geometry import Segment, finite3


class ObservationSurvey:
    def __init__(self, anchor, home, offsets, region, rate, limits, ros):
        self.anchor=finite3(anchor)
        self.targets=[tuple(a+b for a,b in zip(home,finite3(offset))) for offset in offsets]
        if not self.targets or len(self.targets)>4 or not math.isfinite(rate) or rate<=0:
            raise ValueError('INVALID_OBSERVATION_PATTERN')
        if not all(region(p) for p in [self.anchor,*self.targets]):
            raise ValueError('OBSERVATION_OUTSIDE_KNOWN_REGION')
        self.rate,self.limits=rate,limits
        self.index=0;self.angle=0.;self.state='MOVE'
        self.target=self.targets[0]
        self.segment=Segment(self.anchor,self.target,*limits)
        self.started_ros=ros

    def sample(self, ros, now, stable):
        if self.state in ('MOVE','RETURN'):
            elapsed=ros-self.started_ros
            reference=self.segment.at(elapsed)
            if elapsed>=self.segment.duration and stable(self.target,now):
                if self.state=='RETURN':self.state='DONE'
                else:self.state='SCAN';self.scan_started_ros=ros
            return reference,self.angle,self.state=='DONE'
        if self.state=='SCAN':
            sweep=min(2*math.pi,max(0.,ros-self.scan_started_ros)*self.rate)
            self.angle=self.index*2*math.pi+sweep
            if sweep>=2*math.pi:
                self.index+=1
                destination=self.targets[self.index] if self.index<len(self.targets) else self.anchor
                self.state='MOVE' if self.index<len(self.targets) else 'RETURN'
                self.segment=Segment(self.target,destination,*self.limits)
                self.target=destination;self.started_ros=ros
            return self.segment.start if self.state in ('MOVE','RETURN') else self.target,self.angle,False
        return self.anchor,self.angle,True
