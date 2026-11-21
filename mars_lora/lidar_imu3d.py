"""Conservative 3D range sensing + strapdown inertial prediction (research prototype).

Explicitly synthetic: world-frame accelerations originate from finite-differenced
MuJoCo velocity and are corrupted by seeded bias/noise. The 3D LiDAR uses
axis-aligned true scene boxes to generate rays; scan registration receives
ONLY range measurements, the assumed prior map, and the estimated pose/yaw.
There is no image/LiDAR map construction, loop closure, or calibrated sensor.
"""
from __future__ import annotations
import math,random
from dataclasses import dataclass,field

RANGE=24.
AZIMUTHS=12
ELEVATIONS=(-.40,-.14,.14,.40)

def unit_rays(yaw=0.,azimuths=AZIMUTHS,elevations=ELEVATIONS):
    return [(math.cos(pitch)*math.cos(yaw+2*math.pi*i/azimuths),
             math.cos(pitch)*math.sin(yaw+2*math.pi*i/azimuths),
             math.sin(pitch))
            for pitch in elevations for i in range(azimuths)]

def ray_box(origin,direction,box,limit=RANGE):
    lo,hi=0.,limit
    for value,v,center,half in zip(origin,direction,box[:3],box[3:]):
        if abs(v)<1.e-12:
            if abs(value-center)>half:return limit
            continue
        t0,t1=(center-half-value)/v,(center+half-value)/v
        lo=max(lo,min(t0,t1));hi=min(hi,max(t0,t1))
        if lo>hi:return limit
    return max(0.,lo) if lo<=limit and hi>=0 else limit

def scan_at(position,yaw,boxes,limit=RANGE):
    rays=unit_rays(yaw)
    nearby=[box for box in boxes
            if math.dist(position,box[:3])<=limit+math.sqrt(sum(x*x for x in box[3:]))]
    return [min((ray_box(position,r,b,limit) for b in nearby),default=limit)
            for r in rays]

def simulate_scan(position,yaw,boxes,rng,noise=.06,dropout=.03,limit=RANGE):
    if noise<0 or not 0<=dropout<1:raise ValueError("Invalid LiDAR noise")
    truth=scan_at(position,yaw,boxes,limit)
    return [limit if v>=limit-.01 or rng.random()<dropout
            else min(limit,max(0.,v+rng.gauss(0.,noise))) for v in truth]

def register_scan(prior,yaw,ranges,boxes,max_step=.9,limit=RANGE):
    """Conservative XYZ coarse-to-fine scan match; no true pose argument."""
    if len(ranges)!=AZIMUTHS*len(ELEVATIONS) or any(not math.isfinite(x) for x in ranges):
        raise ValueError("Expected 48 finite 3D range measurements")
    observed=[i for i,d in enumerate(ranges) if d<limit-.5]
    if len(observed)<7:return tuple(prior),False,float("inf"),0
    def loss(p):
        predicted=scan_at(p,yaw,boxes,limit)
        return sum(min(4.,(predicted[i]-ranges[i])**2) for i in observed)/len(observed)
    origin=tuple(float(x) for x in prior)
    best=origin
    initial=loss(best);score=initial
    for dx in (-max_step,0.,max_step):
        for dy in (-max_step,0.,max_step):
            for dz in (-.6,0.,.6):
                p=(origin[0]+dx,origin[1]+dy,origin[2]+dz)
                e=loss(p)
                if e<score:score=e;best=p
    # Recenter around the lowest-error coarse-grid candidate.
    coarse=best
    for dx in (-.22,0.,.22):
        for dy in (-.22,0.,.22):
            for dz in (-.16,0.,.16):
                p=(coarse[0]+dx,coarse[1]+dy,coarse[2]+dz)
                e=loss(p)
                if e<score:score=e;best=p
    # Do not trust a scan just because the best candidate beats a poor prior:
    # absolute fit AND improvement must be acceptable.
    accepted=score<.7 and score+.008<initial
    return (best if accepted else tuple(prior)),accepted,score,len(observed)

@dataclass
class InertialMapEstimator:
    """Minimal 3D position/velocity/yaw estimate and scalar uncertainty proxy.

    This is complementary map correction of inertially propagated state, not
    a consistent 15-state EKF. Initialization at a known launch location is
    permitted, no other simulator position is given to predict/correct.
    """
    position:list[float]
    velocity:list[float]=field(default_factory=lambda:[0.,0.,0.])
    yaw:float=0.
    sigma_m:float=.06
    process_sigma_m_sqrt_s:float=.025
    accepted:int=0
    rejected:int=0
    @classmethod
    def initial(cls,launch=(0.,0.,.5),yaw=0.):
        return cls(list(map(float,launch)),yaw=yaw)
    def predict(self,accel_world,gyro_z,dt):
        if dt<=0 or any(not math.isfinite(v) for v in (*accel_world,gyro_z)):
            raise ValueError("Invalid inertial sensor packet")
        self.yaw+=gyro_z*dt
        for axis in range(3):
            old=self.velocity[axis]
            self.velocity[axis]+=accel_world[axis]*dt
            self.position[axis]+=old*dt+.5*accel_world[axis]*dt*dt
        self.sigma_m=min(5.,math.sqrt(self.sigma_m**2+self.process_sigma_m_sqrt_s**2*dt))
    def correct_altitude(self,measured_z,gain=.65):
        """Noisy barometer altimeter update, not simulator truth input."""
        if not math.isfinite(measured_z):
            raise ValueError("Invalid barometer measurement")
        self.position[2]+=gain*(measured_z-self.position[2])
        self.velocity[2]*=.96

    def correct_horizontal_flow(self,measured_vx,measured_vy,gain=.35):
        """Noisy measured horizontal optical-flow velocity, not truth velocity."""
        for axis,observed in enumerate((measured_vx,measured_vy)):
            if not math.isfinite(observed):
                raise ValueError("Invalid optical-flow measurement")
            self.velocity[axis]+=gain*(observed-self.velocity[axis])

    def correct_planar_lidar(self,position_xy,accepted):
        """Fallback 2D scan-to-known-map registration in sparse 3D geometry."""
        if accepted:
            for axis in (0,1):
                self.position[axis]+=.80*(position_xy[axis]-self.position[axis])
            self.sigma_m=max(.06,self.sigma_m*.68)
            self.accepted+=1
        else:
            self.rejected+=1
            self.sigma_m=min(5.,self.sigma_m+.012)

    def correct(self,ranges,boxes):
        candidate,accepted,loss,hits=register_scan(self.position,self.yaw,ranges,boxes)
        if accepted:
            for axis in range(3):
                innovation=candidate[axis]-self.position[axis]
                self.position[axis]+=.82*innovation
                # Dampen inertial drift after a successful map correction.
                self.velocity[axis]*=.92
            self.sigma_m=max(.055,min(.45,.68*self.sigma_m+.025))
            self.accepted+=1
        else:
            self.sigma_m=min(5.,self.sigma_m+.018)
            self.rejected+=1
        return accepted,loss,hits
