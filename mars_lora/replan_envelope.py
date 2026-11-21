"""Conservative *necessary* dynamic handoff checks for map-known UAV replans.

These tests use only estimated pose, velocity, uncertainty, and a static AABB map.
They do not establish a robust controlled invariant set or replace native MuJoCo
contact evaluation. Rejection means no trajectory handoff, not safe hovering.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from mars_lora.rf_astar import signed_clearance


@dataclass(frozen=True)
class EnvelopeResult:
    accepted: bool
    reason: str
    min_predicted_clearance_m: float
    stopping_distance_m: float
    inflation_m: float


def check_handoff(position, velocity, points, solids, *,
                  vehicle_radius_m=.32, localization_sigma_m=.2,
                  required_clearance_m=.25, braking_accel_m_s2=.8,
                  actuator_delay_s=.2, wind_accel_bound_m_s2=.15,
                  max_splice_m=2.8, sample_step_m=.10):
    """Check a delayed-braking corridor and first splice, with conservative margins.

    Every listed quantity is a *declared design assumption*, not identified
    aircraft/sensor uncertainty. Conservative scalar 3-sigma inflation does
    not prove a probabilistic chance constraint.
    """
    values=(*position,*velocity,vehicle_radius_m,localization_sigma_m,
            required_clearance_m,braking_accel_m_s2,actuator_delay_s,
            wind_accel_bound_m_s2,max_splice_m,sample_step_m)
    if (len(position)!=3 or len(velocity)!=3 or
            any(not math.isfinite(v) for v in values)):
        raise ValueError("Expected finite 3D pose, velocity and parameters")
    if (vehicle_radius_m<=0 or localization_sigma_m<0 or required_clearance_m<0 or
            braking_accel_m_s2<=wind_accel_bound_m_s2 or actuator_delay_s<0 or
            wind_accel_bound_m_s2<0 or max_splice_m<=0 or sample_step_m<=0):
        raise ValueError("Invalid reachable-envelope assumptions")
    if not points:
        return EnvelopeResult(False,"no_path",float("-inf"),0,0)
    if any(len(p)!=3 or any(not math.isfinite(v) for v in p) for p in points):
        raise ValueError("Nonfinite replan waypoint")
    p=tuple(float(x) for x in position)
    v=tuple(float(x) for x in velocity)
    inflation=vehicle_radius_m+3*localization_sigma_m+required_clearance_m
    speed=math.sqrt(sum(q*q for q in v))
    effective_brake=braking_accel_m_s2-wind_accel_bound_m_s2
    stop_distance=speed*actuator_delay_s+speed**2/(2*effective_brake)
    # Model forward translation before actuator response, then brake along
    # initial estimated velocity. Wind / tracking errors are represented by
    # conservative scalar margin, not a validated time-varying reachable set.
    if speed>1e-9:
        direction=tuple(q/speed for q in v)
        stop=tuple(p[i]+direction[i]*stop_distance for i in range(3))
    else:
        stop=p
    def checked_segment(a,b):
        length=math.dist(a,b)
        samples=max(1,math.ceil(length/sample_step_m))
        return min(signed_clearance(tuple(a[k]+(b[k]-a[k])*i/samples
                                          for k in range(3)),solids)
                   for i in range(samples+1))
    closest=checked_segment(p,stop)
    if closest<inflation:
        return EnvelopeResult(False,"unsafe_stopping_corridor",
                              closest-inflation,stop_distance,inflation)
    if math.dist(p,points[0])>max_splice_m:
        return EnvelopeResult(False,"splice_too_long",
                              closest-inflation,stop_distance,inflation)
    # Check estimated-position -> snapped grid origin and the *first*
    # commanded leg. A* later legs still need native closed-loop validation.
    chain=[p,*points[:2]]
    for a,b in zip(chain,chain[1:]):
        closest=min(closest,checked_segment(a,b))
        if closest<inflation:
            return EnvelopeResult(False,"unsafe_splice",
                                  closest-inflation,stop_distance,inflation)
    return EnvelopeResult(True,"accepted",closest-inflation,stop_distance,inflation)
