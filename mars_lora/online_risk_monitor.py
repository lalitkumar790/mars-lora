"""Online estimated-state clearance alarm, not a certified collision controller.

The clearance provider must report physical clearance with aircraft radius
already deducted. Scalar uncertainty is a proxy, not a calibrated covariance;
crossing the threshold reports a risk event, not a safety certificate.
"""
from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RiskSignal:
    warning: bool
    reason: str
    predicted_margin_m: float
    stopping_distance_m: float
    estimated_sigma_m: float


def predict_clearance_alarm(position,velocity,sigma_m,clearance_fn,*,
                            required_clearance_m=.25,
                            braking_accel_m_s2=.8,
                            wind_accel_m_s2=.15,latency_s=.2,
                            step_m=.3):
    """Evaluate estimated pose and straight-line nominal stopping sweep.

    Wind is represented as a reduction in available braking. This model
    does not account for 6-DoF rotor saturation or arbitrary wind direction.
    """
    if len(position)!=3 or len(velocity)!=3:
        raise ValueError("Expected 3D estimated position and velocity")
    vals=(*position,*velocity,sigma_m,required_clearance_m,
          braking_accel_m_s2,wind_accel_m_s2,latency_s,step_m)
    if any(not math.isfinite(x) for x in vals):
        return RiskSignal(True,"nonfinite_estimate",float("-inf"),float("inf"),sigma_m)
    if sigma_m<0 or required_clearance_m<0 or wind_accel_m_s2<0 or latency_s<0 or step_m<=0 or braking_accel_m_s2<=wind_accel_m_s2:
        raise ValueError("Invalid uncertainty/braking assumptions")
    speed=math.sqrt(sum(v*v for v in velocity))
    stop=speed*latency_s+speed*speed/(2*(braking_accel_m_s2-wind_accel_m_s2))
    if stop>1000: # avoid unbounded computation under estimator divergence
        return RiskSignal(True,"unbounded_stopping_distance",float("-inf"),stop,sigma_m)
    n=max(1,math.ceil(stop/step_m))
    direction=tuple(v/speed for v in velocity) if speed>1e-8 else (0.,0.,0.)
    lowest=float("inf")
    for i in range(n+1):
        p=tuple(position[j]+direction[j]*stop*i/n for j in range(3))
        lowest=min(lowest,clearance_fn(p))
    margin=lowest-3*sigma_m
    return RiskSignal(margin<required_clearance_m,
                      "uncertain_stopping_corridor" if margin<required_clearance_m else "clear",
                      margin,stop,sigma_m)
