"""Stopped-start EGO execution admission, owned by the sole flight gateway."""
from copy import deepcopy
import math
import numpy as np
from uav_nav_sim.planning_context import PlanningGate, stamp, xyz, transform_parts
from uav_nav_sim.core import spline, validate_trajectory


def trajectory_version_in_context(planned,checked,current):
    return planned<=checked<=current


class BrakingGrid:
    """Observed body volume plus the profile's horizontal braking reserve."""
    def __init__(self, grid, horizontal):
        self.base,self.horizontal=grid,horizontal
        self.resolution=grid.resolution

    def collision(self, point, radius):
        g=self.base
        if g.collision(point,radius):return True
        extent=np.array([radius+self.horizontal,radius+self.horizontal,radius])
        lo=np.floor((np.asarray(point)-extent-g.origin)/g.resolution).astype(int)
        hi=np.floor((np.asarray(point)+extent-g.origin)/g.resolution).astype(int)
        if np.any(lo<0) or np.any(hi>=g.distance.shape):return True
        slices=tuple(slice(a,b+1) for a,b in zip(lo,hi))
        cells=g.distance[slices]
        return not (np.all(g.observed[slices]) and np.all(np.isfinite(cells)&(cells>0)))

    def diagnostics(self, point, radius):
        """Read-only voxel evidence for the exact braking box, grouped by height."""
        g=self.base
        while isinstance(g,BrakingGrid):g=g.base
        extent=np.array([radius+self.horizontal,radius+self.horizontal,radius])
        lo=np.floor((np.asarray(point)-extent-g.origin)/g.resolution).astype(int)
        hi=np.floor((np.asarray(point)+extent-g.origin)/g.resolution).astype(int)
        shape=np.asarray(g.distance.shape)
        clipped_lo=np.maximum(lo,0);clipped_hi=np.minimum(hi,shape-1)
        total=int(np.prod(hi-lo+1));layers=[]
        inside=unknown=occupied=nonfinite=0
        if np.all(clipped_lo<=clipped_hi):
            for z in range(int(clipped_lo[2]),int(clipped_hi[2])+1):
                sl=(slice(clipped_lo[0],clipped_hi[0]+1),slice(clipped_lo[1],clipped_hi[1]+1),z)
                d=g.distance[sl];o=g.observed[sl]
                counts=dict(cells=int(d.size),unknown=int(np.count_nonzero(~o)),
                    observed_nonfinite=int(np.count_nonzero(o & ~np.isfinite(d))),
                    occupied=int(np.count_nonzero(o & np.isfinite(d) & (d<=0))))
                inside+=counts['cells'];unknown+=counts['unknown']
                nonfinite+=counts['observed_nonfinite'];occupied+=counts['occupied']
                layers.append(dict(z_m=float(g.origin[2]+z*g.resolution),**counts))
        index=g.index(point)
        centre_distance=None if index is None else float(g.distance[tuple(index)])
        if centre_distance is not None and not math.isfinite(centre_distance):centre_distance=None
        return dict(total_cells=total,outside_cells=total-inside,unknown=unknown,
            occupied=occupied,observed_nonfinite=nonfinite,centre_distance_m=centre_distance,
            required_centre_distance_m=float(radius+np.sqrt(3)*g.resolution/2),
            layers=layers)


class EgoExecution:
    def __init__(self, alignment, region, radius, limits=(.6,.5,.6), braking_margin=0.):
        if not math.isfinite(braking_margin) or braking_margin<0:raise ValueError('INVALID_BRAKING_MARGIN')
        self.braking_margin=braking_margin
        self.gate=PlanningGate(radius=radius,limits=limits)
        self.alignment,self.region=alignment,region
        self.used=set();self.retire()

    def retire(self):
        self.authorization=None;self.curve=None;self.gate.goal=None
        self.bound_context=None;self.last_checked_version=None

    def ready(self, ros, mono):
        if not self.gate.ready(ros,mono):raise ValueError(self.gate.reason)
        grid=self.gate.grid.base if isinstance(self.gate.grid,BrakingGrid) else self.gate.grid
        self.gate.grid=BrakingGrid(grid,self.braking_margin)
        transform=self.gate.inputs['alignment'][0].map_to_odom
        translation,rotation=transform_parts(transform)
        c,s=math.cos(self.alignment.yaw),math.sin(self.alignment.yaw)
        if (not np.allclose(translation,self.alignment.translation,atol=1e-9,rtol=0) or
                not np.allclose(rotation.as_matrix(),[[c,-s,0],[s,c,0],[0,0,1]],atol=1e-9,rtol=0)):
            raise ValueError('FLIGHT_ALIGNMENT_MISMATCH')
        message=self.gate.inputs['map'][0]
        if not message.static_map and not -.05<=ros-stamp(message.source_stamp)<=2.:
            raise ValueError('STALE_MAP_SOURCE')

    def start(self, goal, authorization, ros, mono):
        self.retire();self.ready(ros,mono)
        # coordinator, root UUID, control UUID, generation, owner, child UUID, step.
        if (len(authorization)!=7 or not authorization[0] or authorization[4]!='TASK' or
                any(len(authorization[i])!=16 or not any(authorization[i]) for i in (1,2,5)) or
                authorization[3]<=0 or authorization[6]<0):raise ValueError('INVALID_CONTROL_AUTHORIZATION')
        if self.gate.inputs['odom'][0].localization_session!=authorization[0]:
            raise ValueError('COORDINATOR_LOCALIZATION_MISMATCH')
        self.gate.dispatch(goal,ros,mono)
        self.authorization=authorization;self.requested_ros=ros
        self.goal=deepcopy(goal)

    def admit(self, bound, authorization, ros, mono):
        if self.authorization is None or authorization!=self.authorization:raise ValueError('CONTROL_SESSION_MISMATCH')
        if self.curve is not None:raise ValueError('TRAJECTORY_ALREADY_BOUND')
        self.ready(ros,mono)
        context=bound.context;g=self.gate;message=g.inputs['map'][0];a=g.inputs['alignment'][0]
        if (not context.valid or context.header.frame_id!='map' or not -.05<=ros-stamp(context.header.stamp)<=.5 or
                context.context_id!=g.context_id or (context.map_id,context.map_epoch)!=(message.map_id,message.epoch) or
                context.localization_session!=authorization[0] or context.alignment_id!=a.alignment_id or
                context.alignment_generation!=a.generation or tuple(context.reset_counters)!=tuple(a.reset_counters) or
                not trajectory_version_in_context(bound.trajectory.map_version,context.map_version,message.version)):raise ValueError('EXECUTION_CONTEXT_MISMATCH')
        trajectory=bound.trajectory
        key=(context.localization_session,trajectory.trajectory_id)
        if key in self.used or len(self.used)>=256:raise ValueError('TRAJECTORY_REPLAY_OR_LIMIT')
        # Recheck the latest grid at the gateway; a valid flag is insufficient.
        candidate=spline([xyz(p) for p in trajectory.control_points],trajectory.knot_interval)
        if float(candidate.t[-4])>45.:raise ValueError('EXECUTION_DURATION_LIMIT')
        if any(not self.region(self.alignment.to_odom(p)) for p in candidate.c):raise ValueError('CURVE_OUTSIDE_FLIGHT_REGION')
        checked=g.bind(trajectory,ros,mono)
        self.curve=candidate;self.start_ros=stamp(checked.start_time);self.duration=float(candidate.t[-4])
        self.bound_context=g.context_id;self.trajectory_id=checked.trajectory_id
        self.last_checked_version=message.version;self.used.add(key)

    def sample(self, authorization, ros, mono):
        try:
            if self.authorization is None or authorization!=self.authorization:raise ValueError('CONTROL_SESSION_MISMATCH')
            self.ready(ros,mono)
            if self.bound_context is not None and self.gate.context_id!=self.bound_context:raise ValueError('EXECUTION_CONTEXT_RETIRED')
            if self.curve is None:
                if ros-self.requested_ros>2.:raise ValueError('PLANNER_TIMEOUT')
                return None,False
            elapsed=min(self.duration,max(0.,ros-self.start_ros))
            message=self.gate.inputs['map'][0]
            if message.version!=self.last_checked_version:
                validate_trajectory(self.curve,self.gate.grid,self.gate.radius,self.gate.limits,start=elapsed)
                self.last_checked_version=message.version
            return self.alignment.to_odom(self.curve(elapsed)),ros>=self.start_ros+self.duration
        except ValueError:
            self.retire();raise
