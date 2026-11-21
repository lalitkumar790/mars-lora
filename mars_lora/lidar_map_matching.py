"""GPS-free, map-known 2D LiDAR localization prototype, NOT full SLAM.

Range rays are horizontal and use a known map of axis-aligned 3D solids.
Heading and altitude are taken from the simulated attitude/altitude channels.
The only information used to correct XY is a simulated 2D range scan and
the occupancy map; never the true XY position supplied to the optimizer.

Limitations: perfect prior map, no moving objects/loop closure, simplified
ray geometry, no inertial bias estimation or covariance consistency guarantee.
"""
from __future__ import annotations
import math,random

BEAMS=16
RANGE_M=25.

def ray_box_2d(origin,direction,box,max_range=RANGE_M):
    x,y=origin
    dx,dy=direction
    cx,cy,cz,hx,hy,hz=box
    lo,hi=0.,max_range
    for p,v,center,half in ((x,dx,cx,hx),(y,dy,cy,hy)):
        if abs(v)<1e-12:
            if abs(p-center)>half:return max_range
        else:
            t0=(center-half-p)/v
            t1=(center+half-p)/v
            lo=max(lo,min(t0,t1))
            hi=min(hi,max(t0,t1))
            if lo>hi:return max_range
    return max(0.,min(max_range,lo if lo>0. else hi))

def predict_scan(xy,yaw,z,solids,n_beams=BEAMS,max_range=RANGE_M):
    local=[box for box in solids
           if box[2]-box[5]<=z<=box[2]+box[5] and
           abs(box[0]-xy[0])<max_range+box[3] and
           abs(box[1]-xy[1])<max_range+box[4]]
    ranges=[]
    for i in range(n_beams):
        theta=yaw+2*math.pi*i/n_beams
        direction=(math.cos(theta),math.sin(theta))
        ranges.append(min((ray_box_2d(xy,direction,b,max_range)
                           for b in local),default=max_range))
    return ranges

def observe_scan(true_xy,yaw,z,solids,rng:random.Random,noise_sigma=.06,
                 n_beams=BEAMS,max_range=RANGE_M):
    """Only this sensor-simulation function accesses truth XY."""
    ranges=predict_scan(true_xy,yaw,z,solids,n_beams,max_range)
    return [max(0.,min(max_range,r+noise_sigma*rng.gauss(0.,1.)))
            if r<max_range-.01 else max_range for r in ranges]

def match_scan(prior_xy,yaw,z,readings,solids,max_range=RANGE_M):
    """Bounded coarse-to-fine XY scan matching around a dead-reckoned prior.

    Returns corrected estimate, accepted flag, and scan-fit score. The optimizer
    never receives ground-truth XY, only ranges, map, heading and prior.
    """
    if len(readings)<8 or any(not math.isfinite(r) for r in readings):
        raise ValueError("Invalid LiDAR scan")
    hits=sum(r<max_range-.4 for r in readings)
    if hits<4:return tuple(prior_xy),False,float("inf")
    def error(x,y):
        pred=predict_scan((x,y),yaw,z,solids,len(readings),max_range)
        # Use only observed returns, with Huber-like clipping to tolerate
        # occasional unmodeled points.
        residuals=[min(9.,(p-o)**2) for p,o in zip(pred,readings)
                   if o<max_range-.4]
        return sum(residuals)/max(1,len(residuals))
    best=(float(prior_xy[0]),float(prior_xy[1]))
    original_score=error(*best)
    best_score=original_score
    # Low-cost local search: never jump farther than 1.5m at one update.
    for dx in (-1.5,-1.,-.5,0.,.5,1.,1.5):
        for dy in (-1.5,-1.,-.5,0.,.5,1.,1.5):
            xy=(prior_xy[0]+dx,prior_xy[1]+dy)
            val=error(*xy)
            if val<best_score:
                best_score=val;best=xy
    coarse=best
    for dx in (-.3,-.15,0.,.15,.3):
        for dy in (-.3,-.15,0.,.15,.3):
            xy=(coarse[0]+dx,coarse[1]+dy)
            val=error(*xy)
            if val<best_score:
                best_score=val;best=xy
    # Discard scans that are uninformative or inconsistent with our static
    # map instead of trusting a spurious optimization minimum.
    accepted=(best_score<=1.2 and best_score+0.005<original_score)
    return (tuple(best) if accepted else tuple(prior_xy)),accepted,best_score
