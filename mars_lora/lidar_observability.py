"""Local 3D LiDAR translation observability diagnostics (known-map simulation).

Computes a numerical range Jacobian at the *estimated pose*, not ground truth.
No filter update or control authority. Rank, condition and information
eigenvalues describe only a local linearized, selected-hit measurement model.
They are NOT a validated SLAM covariance or a physical safety certificate.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from mars_lora.lidar_imu3d import scan_at, RANGE

@dataclass(frozen=True)
class ScanObservability:
    valid_hits:int
    used_hits:int
    geometric_rank:int
    condition_number:float
    smallest_info_eigenvalue:float
    largest_info_eigenvalue:float
    rms_range_residual_m:float
    informative:bool


def assess_scan_observability(position_xyz,yaw,ranges,solids,*,
                             range_noise_m=.06,delta_m=.08,
                             min_hits=7,min_eigenvalue=2.,
                             max_condition=1000.):
    p=np.asarray(position_xyz,dtype=float)
    ranges=np.asarray(ranges,dtype=float)
    if (p.shape!=(3,) or ranges.shape!=(48,) or
        not np.all(np.isfinite(p)) or not np.all(np.isfinite(ranges)) or
        not math.isfinite(yaw) or not math.isfinite(range_noise_m) or
        range_noise_m<=0 or not math.isfinite(delta_m) or delta_m<=0 or
        min_hits<1 or min_eigenvalue<0 or max_condition<=1):
        raise ValueError("Invalid 3D scan diagnostics inputs")
    valid=np.where((ranges>=0)&(ranges<RANGE-.5))[0]
    if valid.size<min_hits:
        return ScanObservability(int(valid.size),0,0,float("inf"),0.,0.,
                                 float("inf"),False)
    expected=np.asarray(scan_at(tuple(p),yaw,solids),dtype=float)
    # Reject rays whose *expected* return is absent: discontinuous ray-box
    # transitions are not a valid smooth Jacobian constraint.
    selected=np.array([i for i in valid if expected[i]<RANGE-.5],dtype=int)
    if selected.size<min_hits:
        return ScanObservability(int(valid.size),int(selected.size),0,
                                 float("inf"),0.,0.,float("inf"),False)
    J=np.zeros((len(selected),3))
    smooth=np.ones(len(selected),dtype=bool)
    for axis in range(3):
        step=np.zeros(3);step[axis]=delta_m
        plus=np.asarray(scan_at(tuple(p+step),yaw,solids),dtype=float)[selected]
        minus=np.asarray(scan_at(tuple(p-step),yaw,solids),dtype=float)[selected]
        smooth&=(plus<RANGE-.5)&(minus<RANGE-.5)
        J[:,axis]=(plus-minus)/(2*delta_m)
    smooth&=np.all(np.isfinite(J),axis=1)
    J=J[smooth]
    indices=selected[smooth]
    if J.shape[0]<min_hits:
        return ScanObservability(int(valid.size),int(J.shape[0]),0,
                                 float("inf"),0.,0.,float("inf"),False)
    residual=expected[indices]-ranges[indices]
    rms=float(math.sqrt(float(np.mean(residual**2))))
    info=(J.T@J)/(range_noise_m**2)
    eig=np.maximum(0.,np.linalg.eigvalsh(info))
    rank=int(np.sum(eig>max(1e-8,1e-8*eig[-1])))
    condition=float(eig[-1]/eig[0]) if eig[0]>1e-10 else float("inf")
    informative=bool(rank==3 and eig[0]>=min_eigenvalue and
                     condition<=max_condition)
    return ScanObservability(int(valid.size),len(indices),rank,
                             condition,float(eig[0]),float(eig[-1]),rms,
                             informative)
