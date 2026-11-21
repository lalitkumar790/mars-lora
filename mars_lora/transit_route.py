"""Experimental departure/transit/arrival altitude staging.

This is a static map-known route ablation, not a verified UAV safety controller.
The planner must *not* claim the aircraft will perfectly track these segments.
"""
from __future__ import annotations
import math
from mars_lora.rf_astar import plan,path_clearance,START,GOAL


def staged_route(method, solids, *, start=START, goal=GOAL,
                 transit_z=10., radius=.32, margin=.85,
                 localization_sigma_m=.10):
    """Vertical climb, 3D A* transit, vertical descent; reject unsafe legs.

    Intended to avoid entering under-beam terrain while altitude control
    tracks with overshoot. May be slower or consume additional energy.
    """
    if not math.isfinite(transit_z) or transit_z<7.:
        raise ValueError("Transit altitude must be finite and >= 7 m")
    lifted_start=(float(start[0]),float(start[1]),float(transit_z))
    lifted_goal=(float(goal[0]),float(goal[1]),float(transit_z))
    route=plan(method,solids=solids,start_xyz=lifted_start,
               goal_xyz=lifted_goal,radius=radius,margin=margin,
               localization_sigma_m=localization_sigma_m,max_nodes=16000)
    points=[tuple(map(float,start)),lifted_start,*route.points,lifted_goal,tuple(map(float,goal))]
    compact=[]
    for p in points:
        if not compact or math.dist(p,compact[-1])>1.e-6:compact.append(p)
    # Check all segments including ungridded takeoff/landing vertical legs.
    required=margin+3*localization_sigma_m
    clearance=path_clearance(compact,radius=radius,margin=required,solids=solids)
    if clearance<required-1.e-6:
        raise ValueError("Staged route violates static map clearance")
    return compact
