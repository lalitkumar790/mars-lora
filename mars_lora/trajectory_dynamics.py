"""Phase 12 map-known trajectory candidate with explicit feasibility and timing.

NOT a dynamically feasible *certified* trajectory: bounds are design assumptions
for the heavy MuJoCo craft, not identified closed-loop reachable sets.
Unknown-map mapping, consistent covariance, and real sensor integration are absent.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from mars_lora.rf_astar import plan, path_clearance, signed_clearance, START, GOAL

@dataclass(frozen=True)
class TimedRoute:
    points: tuple[tuple[float,float,float],...]
    waypoint_speed_limits_m_s: tuple[float,...]
    arrival_times_s: tuple[float,...]
    min_static_clearance_m: float
    max_speed_m_s: float
    minimum_margin_m: float
    planning_attempts: int


def _segment_clear(a,b,solids,radius,margin):
    if math.dist(a,b)<1.e-9:
        return signed_clearance(a,solids)>=radius+margin
    return path_clearance((a,b),radius=radius,solids=solids)>=margin-1.e-8


def _anchors(endpoint,solids,radius,margin,transit_z):
    # Keep the exact mission endpoint; search over nearby legally connected
    # launch/approach points rather than assuming a vertical climb is possible.
    ex,ey,ez=endpoint
    offsets=((0,0),(3,0),(-3,0),(0,3),(0,-3),
             (6,0),(-6,0),(0,6),(0,-6),
             (3,3),(-3,-3),(3,-3),(-3,3))
    viable=[]
    for dx,dy in offsets:
        # Approach via a lower-altitude footpoint, then a vertical column.
        foot=(ex+dx,ey+dy,ez)
        upper=(ex+dx,ey+dy,transit_z)
        if (_segment_clear(endpoint,foot,solids,radius,margin) and
            _segment_clear(foot,upper,solids,radius,margin)):
            viable.append((math.hypot(dx,dy),foot,upper))
    viable.sort(key=lambda item:item[0])
    return viable


def build_trajectory(method,solids,*,start=START,goal=GOAL,
                     altitude_candidates=(10.,13.,17.,21.,26.),
                     radius=.32,sigma_m=.12,geometric_margin_m=.55,
                     max_speed_m_s=1.0,braking_m_s2=.8,
                     motor_latency_s=.2,wind_accel_bound_m_s2=.15,
                     corner_speed_m_s=.30,max_pair_attempts=50):
    """Produce static collision-checked approach/transit/exit and speed timetable.

    Every candidate starts/ends at the *exact* requested pose. Segment
    clearance includes 3*sigma, conservative braking distance and latency
    from the declared maximum speed. Failure is explicit (never reclassified
    as a safe mission). RF preference enters only through known-map A*.
    """
    params=(*start,*goal,*altitude_candidates,radius,sigma_m,
            geometric_margin_m,max_speed_m_s,braking_m_s2,
            motor_latency_s,wind_accel_bound_m_s2,corner_speed_m_s)
    if (len(start)!=3 or len(goal)!=3 or
        any(not math.isfinite(x) for x in params) or
        not altitude_candidates or radius<=0 or sigma_m<0 or
        geometric_margin_m<0 or max_speed_m_s<=0 or
        braking_m_s2<=wind_accel_bound_m_s2 or wind_accel_bound_m_s2<0 or
        motor_latency_s<0 or corner_speed_m_s<=0 or
        max_pair_attempts<1):
        raise ValueError('Invalid trajectory boundary or dynamics assumptions')
    if math.dist(start,goal)<1.e-6:
        raise ValueError('Mission endpoints are identical')
    required=geometric_margin_m+3*sigma_m
    if (signed_clearance(start,solids)<radius+required or
        signed_clearance(goal,solids)<radius+required):
        raise ValueError('Unsafe exact mission start or goal')
    # Rather than treating every point as safe at max speed, cap route speed
    # based on its narrowest clearance; guard adds braking + actuation lag.
    effort=braking_m_s2-wind_accel_bound_m_s2
    attempts=0
    feasible=[]
    for alt in altitude_candidates:
        if alt<=max(start[2],goal[2]):
            continue
        starts=_anchors(start,solids,radius,required,alt)
        goals=_anchors(goal,solids,radius,required,alt)
        # Deterministic bounded paired search, with same-coordinate vertical
        # staging preferred over longer low-altitude lateral motion.
        pairs=sorted(((a[0]+b[0],a,b) for a in starts[:7] for b in goals[:7]),
                     key=lambda item:item[0])
        for _,sa,ga in pairs[:max_pair_attempts]:
            attempts+=1
            try:
                transit=plan(method,solids=solids,start_xyz=sa[2],goal_xyz=ga[2],
                    radius=radius,margin=geometric_margin_m,
                    localization_sigma_m=sigma_m,max_nodes=16000)
            except (ValueError,RuntimeError,AssertionError):
                continue
            candidate=[tuple(start),sa[1],sa[2],*transit.points,ga[2],ga[1],tuple(goal)]
            points=[]
            for p in candidate:
                p=tuple(map(float,p))
                if not points or math.dist(points[-1],p)>1.e-7:
                    points.append(p)
            # Check *all* continuous connectors, including lattice snapping.
            clearance=path_clearance(points,radius=radius,solids=solids)
            if clearance<required-1.e-6:
                continue
            available=max(0.,clearance-required)
            # Delay + stopping offset must remain within the unconsumed map
            # margin under the assumed deceleration/wind bound.
            lo,hi=0.,max_speed_m_s
            for _ in range(25):
                mid=(lo+hi)/2
                if mid*motor_latency_s+mid*mid/(2*effort)<=available:
                    lo=mid
                else:
                    hi=mid
            speed=min(max_speed_m_s,lo)
            if speed<.18: # infeasible tracking speed, not an admissible path
                continue
            speed_caps=[0.]*len(points)
            speed_caps[0]=0.;speed_caps[-1]=0.
            for i in range(1,len(points)-1):
                a=tuple(points[i][j]-points[i-1][j] for j in range(3))
                b=tuple(points[i+1][j]-points[i][j] for j in range(3))
                an=math.sqrt(sum(v*v for v in a));bn=math.sqrt(sum(v*v for v in b))
                turn=0. if an<1.e-8 or bn<1.e-8 else math.acos(max(-1.,min(1.,
                    sum(a[j]*b[j] for j in range(3))/(an*bn))))
                speed_caps[i]=min(speed,corner_speed_m_s if turn>.35 else speed)
            # Acceleration- and braking-bounded feedforward speed limits.
            for i in range(1,len(points)):
                d=math.dist(points[i-1],points[i])
                speed_caps[i]=min(speed_caps[i],math.sqrt(speed_caps[i-1]**2+2*effort*d))
            for i in range(len(points)-2,-1,-1):
                d=math.dist(points[i+1],points[i])
                speed_caps[i]=min(speed_caps[i],math.sqrt(speed_caps[i+1]**2+2*effort*d))
            times=[0.]
            for i in range(1,len(points)):
                d=math.dist(points[i-1],points[i])
                average=max(.10,min(speed,(speed_caps[i-1]+speed_caps[i])/2))
                times.append(times[-1]+d/average)
            if times[-1]>240.:
                continue
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            feasible.append((length,transit.objective,TimedRoute(
                tuple(points),tuple(speed_caps),tuple(times),clearance,
                speed,required,attempts)))
            # Avoid choosing only nominal shortest when a wide-clearance
            # alternative exists. Keep candidate count bounded for CI.
            if len(feasible)>=8:
                break
        if len(feasible)>=8:
            break
    if not feasible:
        raise RuntimeError('No dynamically screened exact-endpoint trajectory')
    feasible.sort(key=lambda row:(row[0]+.05*row[1],-row[2].min_static_clearance_m))
    chosen=feasible[0][2]
    return TimedRoute(chosen.points,chosen.waypoint_speed_limits_m_s,
        chosen.arrival_times_s,chosen.min_static_clearance_m,
        chosen.max_speed_m_s,chosen.minimum_margin_m,attempts)
