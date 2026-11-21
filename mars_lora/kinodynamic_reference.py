"""Stop-at-corner quintic reference with measurable translational bounds.

This constructs a *point-mass* reference only. It does NOT assert MuJoCo rotor
torque/thrust feasibility, wind rejection, yaw dynamics, or closed-loop tracking
safety. A full UAV feasibility proof needs flatness inversion and constraints.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from mars_lora.rf_astar import path_clearance


@dataclass(frozen=True)
class ReferenceSample:
    t_s: float
    xyz: tuple[float,float,float]
    velocity_xyz: tuple[float,float,float]
    acceleration_xyz: tuple[float,float,float]
    jerk_xyz: tuple[float,float,float]


@dataclass(frozen=True)
class KinodynamicReference:
    samples: tuple[ReferenceSample,...]
    duration_s: float
    measured_max_speed_m_s: float
    measured_max_accel_m_s2: float
    measured_max_jerk_m_s3: float
    minimum_clearance_m: float


def quintic_reference(points,solids,*,radius_m=.32,sigma_m=.12,
                      clearance_margin_m=.25,max_speed_m_s=1.0,
                      max_accel_m_s2=.8,max_jerk_m_s3=1.5,
                      sample_step_s=.10,max_duration_s=240.):
    """Zero-velocity, zero-acceleration at every waypoint.

    f(u)=10u³-15u⁴+6u⁵. Its maximum dimensionless first, second and third
    derivatives are 1.875, 5.773502692 and 60 respectively. Choose each
    segment duration so these translational limits hold analytically.
    """
    if len(points)<2:
        raise ValueError("At least two positions required")
    params=(radius_m,sigma_m,clearance_margin_m,max_speed_m_s,max_accel_m_s2,
            max_jerk_m_s3,sample_step_s,max_duration_s)
    if (any(not math.isfinite(x) for x in params) or
        any(x<=0 for x in (radius_m,max_speed_m_s,max_accel_m_s2,
                           max_jerk_m_s3,sample_step_s,max_duration_s)) or
        sigma_m<0 or clearance_margin_m<0):
        raise ValueError("Invalid reference constraints")
    p=[tuple(float(x) for x in pt) for pt in points]
    if any(len(pt)!=3 or any(not math.isfinite(x) for x in pt) for pt in p):
        raise ValueError("Expected finite 3D points")
    distances=[math.dist(a,b) for a,b in zip(p,p[1:])]
    if any(d<=1e-8 for d in distances):
        raise ValueError("Consecutive positions must differ")
    margin=3*sigma_m+clearance_margin_m
    clearance=path_clearance(p,radius=radius_m,solids=solids)
    if clearance<margin-1.e-7:
        raise ValueError("Unsafe reference polyline under uncertainty budget")
    samples=[]
    time=0.
    peakv=peaka=peakj=0.
    for index,(a,b) in enumerate(zip(p,p[1:])):
        d=distances[index]
        T=max(1.875*d/max_speed_m_s,
              math.sqrt(5.773502692*d/max_accel_m_s2),
              (60*d/max_jerk_m_s3)**(1/3))
        if time+T>max_duration_s:
            raise RuntimeError("Bounded reference exceeds mission duration")
        direction=[b[k]-a[k] for k in range(3)]
        count=max(2,math.ceil(T/sample_step_s))
        # Avoid duplicate time/positions at segment joins.
        for i in range(count+1):
            if index>0 and i==0:
                continue
            u=i/count
            s=10*u**3-15*u**4+6*u**5
            ds=30*u*u-60*u**3+30*u**4
            dds=60*u-180*u*u+120*u**3
            ddds=60-360*u+360*u*u
            pos=tuple(a[k]+direction[k]*s for k in range(3))
            vel=tuple(direction[k]*ds/T for k in range(3))
            acc=tuple(direction[k]*dds/T**2 for k in range(3))
            jerk=tuple(direction[k]*ddds/T**3 for k in range(3))
            peakv=max(peakv,math.sqrt(sum(x*x for x in vel)))
            peaka=max(peaka,math.sqrt(sum(x*x for x in acc)))
            peakj=max(peakj,math.sqrt(sum(x*x for x in jerk)))
            samples.append(ReferenceSample(time+T*u,pos,vel,acc,jerk))
        time+=T
    return KinodynamicReference(tuple(samples),time,peakv,peaka,peakj,
                                 clearance)
