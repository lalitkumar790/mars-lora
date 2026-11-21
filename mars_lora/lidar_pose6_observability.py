"""Full 6-DoF *local* geometric observability of a hypothetical body-fixed 48-ray LiDAR.

The production/native legacy LiDAR is yaw-only. This file deliberately models
an ADDITIONAL separate sensor whose ray directions rotate with full vehicle
attitude. Mixing legacy yaw-only scans with this model is INVALID.

Rank concerns only local range measurement geometry at the estimated pose;
it cannot establish global pose uniqueness, map correctness, robust SLAM or
flight safety. Translation and rotation columns are scaled to state units.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from mars_lora.lidar_imu3d import RANGE,unit_rays,ray_box
from mars_lora.body_imu_eskf15 import normalize_quat,quat_to_rot,qmul,exp_quat

_BODY_RAYS=np.asarray(unit_rays(0.),dtype=float)


def body_scan(position_xyz,quaternion_wb,boxes,limit=RANGE):
    p=np.asarray(position_xyz,dtype=float)
    if p.shape!=(3,) or not np.all(np.isfinite(p)):
        raise ValueError("Expected finite XYZ")
    rot=quat_to_rot(quaternion_wb)
    directions=_BODY_RAYS@rot.T
    nearby=[b for b in boxes
            if math.dist(tuple(p),b[:3]) <= limit+
                math.sqrt(sum(x*x for x in b[3:]))]
    return np.array([min((ray_box(p,d,b,limit) for b in nearby),
                        default=limit) for d in directions],dtype=float)


def simulate_body_scan(position_xyz,quaternion_wb,boxes,rng,*,
                       noise=.06,dropout=.03):
    if not 0<=dropout<1 or noise<0:
        raise ValueError("Invalid scan noise/dropout")
    ideal=body_scan(position_xyz,quaternion_wb,boxes)
    return [RANGE if distance>=RANGE-.01 or rng.random()<dropout
            else min(RANGE,max(0.,float(distance)+rng.gauss(0.,noise)))
            for distance in ideal]


@dataclass(frozen=True)
class PoseObservability:
    hits:int
    usable:int
    rank6:int
    translation_rank:int
    rotation_rank:int
    information_min:float
    information_max:float
    condition_number:float
    residual_rmse_m:float
    locally_full_rank:bool


def assess_pose6(position_xyz,quaternion_wb,ranges,solids,*,
                 translation_step_m=.06,rotation_step_rad=.012,
                 angular_scale_m=2.,noise_std_m=.06,min_hits=7,
                 rank_ratio=1e-5,max_condition=1e5):
    """Right-multiplicative body angular perturbations, local 6D Jacobian.

    Convert angular perturbations to equivalent displacement units using
    angular_scale_m, and report local rank relative to maximum eigenvalue.
    Absolute information eigenvalues depend on the fixed scale convention.
    """
    p=np.asarray(position_xyz,dtype=float)
    q=normalize_quat(quaternion_wb)
    z=np.asarray(ranges,dtype=float)
    scalars=(translation_step_m,rotation_step_rad,angular_scale_m,noise_std_m,
             rank_ratio,max_condition)
    if (p.shape!=(3,) or not np.all(np.isfinite(p)) or
        z.shape!=(len(_BODY_RAYS),) or not np.all(np.isfinite(z)) or
        any(not math.isfinite(a) or a<=0 for a in scalars) or min_hits<1):
        raise ValueError("Invalid sensor data or finite-difference settings")
    valid=np.where((z>=0)&(z<RANGE-.5))[0]
    missing=PoseObservability(len(valid),0,0,0,0,0.,0.,float("inf"),
                              float("inf"),False)
    if len(valid)<min_hits:return missing
    nominal=body_scan(p,q,solids)
    selected=valid[nominal[valid]<RANGE-.5]
    if len(selected)<min_hits:return missing
    jac=np.zeros((len(selected),6),dtype=float)
    smooth=np.ones(len(selected),dtype=bool)
    for axis in range(6):
        if axis<3:
            d=np.zeros(3);d[axis]=translation_step_m
            plus=body_scan(p+d,q,solids)[selected]
            minus=body_scan(p-d,q,solids)[selected]
            jac[:,axis]=(plus-minus)/(2*translation_step_m)
        else:
            d=np.zeros(3);d[axis-3]=rotation_step_rad
            plus=body_scan(p,qmul(q,exp_quat(d)),solids)[selected]
            minus=body_scan(p,qmul(q,exp_quat(-d)),solids)[selected]
            jac[:,axis]=(plus-minus)/(2*rotation_step_rad*angular_scale_m)
        smooth&=(plus<RANGE-.5)&(minus<RANGE-.5)
    indices=selected[smooth]
    jac=jac[smooth]
    if len(indices)<min_hits:return PoseObservability(
        len(valid),len(indices),0,0,0,0.,0.,float("inf"),float("inf"),False)
    residual=nominal[indices]-z[indices]
    rms=float(np.sqrt(np.mean(residual**2)))
    # The selected returns themselves can include false map associations.
    information=jac.T@jac/noise_std_m**2
    values=np.maximum(0.,np.linalg.eigvalsh(information))
    def rank(mat):
        eig=np.maximum(0.,np.linalg.eigvalsh(mat))
        return int(np.sum(eig>max(1e-8,rank_ratio*max(eig[-1],1e-10))))
    rank6=rank(information)
    t_rank=rank(information[:3,:3])
    r_rank=rank(information[3:,3:])
    condition=float(values[-1]/values[0]) if values[0]>1e-10 else float("inf")
    return PoseObservability(len(valid),len(indices),rank6,t_rank,r_rank,
           float(values[0]),float(values[-1]),condition,rms,
           bool(rank6==6 and condition<=max_condition))
