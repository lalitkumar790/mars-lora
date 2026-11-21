"""Phase 17: conservative *map-oracle* observability-biased waypoint insertion.

This is a development hypothesis test, NOT on-line active SLAM: all scores
are computed with an exact known map and a nominal LEVEL body-fixed LiDAR.
The existing UAV controller continues to follow the same type of quintic
trajectory and still receives only the old yaw-only LiDAR stream.

No inserted waypoint is accepted without whole-route static clearance and
the same quintic speed/acceleration/jerk/deadline checks as the baseline.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from mars_lora.body_imu_eskf15 import exp_quat
from mars_lora.lidar_pose6_observability import body_scan,assess_pose6
from mars_lora.rf_astar import path_clearance
from mars_lora.kinodynamic_reference import quintic_reference

@dataclass(frozen=True)
class ViewpointPlan:
    points:tuple[tuple[float,float,float],...]
    baseline_informative_fraction:float
    candidate_informative_fraction:float
    baseline_length_m:float
    candidate_length_m:float
    baseline_duration_s:float
    candidate_duration_s:float
    inserted_waypoint:tuple[float,float,float]|None
    candidate_count:int
    rejected_count:int
    outcome:str


def _route_length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


def _score_route(points,solids,*,spacing_m=8.):
    # Yaw aligned with route horizontal tangent; roll and pitch fixed at 0.
    # A real UAV rolls/pitches in wind and these scores can change.
    total=good=0
    for a,b in zip(points,points[1:]):
        d=math.dist(a,b)
        n=max(1,math.ceil(d/spacing_m))
        yaw=math.atan2(b[1]-a[1],b[0]-a[0]) if math.hypot(
            b[1]-a[1],b[0]-a[0])>.01 else 0.
        q=exp_quat((0.,0.,yaw))
        for i in range(n):
            u=(i+.5)/n
            p=tuple(a[j]+u*(b[j]-a[j]) for j in range(3))
            scan=body_scan(p,q,solids)
            geometry=assess_pose6(p,q,scan,solids)
            total+=1
            good+=int(geometry.locally_full_rank)
    return good/max(1,total)


def choose_observable_detour(points,solids,*,radius_m=.32,
                             sigma_m=.10,clearance_margin_m=.55,
                             max_reference_s=210.,
                             max_extra_length_m=15.,
                             minimum_gain=.05):
    """Return baseline if no safe, deadline-feasible observability gain.

    This test is *not* a global optimizer. It explores one waypoint
    insertion into up to three longest horizontal portions of the route.
    """
    points=tuple(tuple(float(x) for x in p) for p in points)
    if len(points)<2 or not all(len(p)==3 and all(math.isfinite(v)
                                      for v in p) for p in points):
        raise ValueError("Expected finite 3D route")
    if any(not math.isfinite(x) or x<0 for x in (
            radius_m,sigma_m,clearance_margin_m,max_reference_s,
            max_extra_length_m,minimum_gain)):
        raise ValueError("Invalid routing constraints")
    def reference(route):
        return quintic_reference(route,solids,radius_m=radius_m,
             sigma_m=sigma_m,clearance_margin_m=clearance_margin_m,
             max_speed_m_s=1.,max_accel_m_s2=.8,max_jerk_m_s3=1.5,
             max_duration_s=max_reference_s)
    base_ref=reference(points)
    base_score=_score_route(points,solids)
    base_length=_route_length(points)
    # Skip initial ascent/final descent when available; do not bias the
    # candidate toward a launch/landing column close to the ground.
    middle=[(i,math.dist(a,b)) for i,(a,b) in
            enumerate(zip(points,points[1:]))
            if math.hypot(b[0]-a[0],b[1]-a[1])>2.]
    middle.sort(key=lambda x:-x[1])
    best=None
    attempts=rejects=0
    for i,_ in middle[:3]:
        a,b=points[i:i+2]
        dx=b[0]-a[0];dy=b[1]-a[1]
        norm=math.hypot(dx,dy)
        nx,ny=(-dy/norm,dx/norm)
        midpoint=tuple((a[k]+b[k])/2 for k in range(3))
        for offset in (-6.,-3.,3.,6.):
            candidate=(midpoint[0]+offset*nx,
                       midpoint[1]+offset*ny,midpoint[2])
            route=points[:i+1]+(candidate,)+points[i+1:]
            attempts+=1
            if _route_length(route)-base_length>max_extra_length_m:
                rejects+=1;continue
            if path_clearance(route,radius=radius_m,solids=solids)<(
                    3*sigma_m+clearance_margin_m):
                rejects+=1;continue
            try:
                ref=reference(route)
            except (ValueError,RuntimeError):
                rejects+=1;continue
            score=_score_route(route,solids)
            if score<base_score+minimum_gain:
                rejects+=1;continue
            # Enforce positive measured gain, prefer short/durable detours.
            utility=(score-base_score)-.002*(_route_length(route)-base_length)
            if best is None or utility>best[0]:
                best=(utility,route,score,ref,candidate)
    if best is None:
        return ViewpointPlan(points,base_score,base_score,base_length,
                             base_length,base_ref.duration_s,
                             base_ref.duration_s,None,attempts,rejects,
                             "baseline_no_verified_gain")
    _,chosen,score,ref,inserted=best
    return ViewpointPlan(chosen,base_score,score,base_length,
                         _route_length(chosen),base_ref.duration_s,
                         ref.duration_s,inserted,attempts,rejects,
                         "map_oracle_detour_candidate")
