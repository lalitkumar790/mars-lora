"""Timed synthetic body-frame IMU derived from MuJoCo truth for *sensor simulation only*.

MuJoCo free-joint qpos stores wxyz orientation; qvel translational velocity
is world-frame. We use consecutive *quaternion* measurements, NOT assumed
angular-qvel conventions, to estimate integrated body gyro samples.
No ground-truth pose/velocity is consumed by the estimator after initialization.
"""
from __future__ import annotations
import math,random
from dataclasses import dataclass
import numpy as np
from mars_lora.body_imu_eskf15 import (normalize_quat,quat_to_rot,
                                        quat_body_delta,vec3)

@dataclass(frozen=True)
class IMUPacket:
    timestamp_s:float
    dt_s:float
    specific_force_b:tuple[float,float,float]
    angular_rate_b:tuple[float,float,float]

class BodyIMUSurrogate:
    def __init__(self,seed=0,accel_noise_std=.025,gyro_noise_std=.003,
                 accel_bias_std=.003,gyro_bias_std=.0008):
        self.rng=random.Random(seed)
        self.accel_bias=np.array([self.rng.gauss(0,accel_bias_std) for _ in range(3)])
        self.gyro_bias=np.array([self.rng.gauss(0,gyro_bias_std) for _ in range(3)])
        self.accel_noise_std=accel_noise_std
        self.gyro_noise_std=gyro_noise_std
        self.previous_t=None
        self.previous_v=None
        self.previous_q=None

    def reset(self,t,velocity_world,quat_wb):
        if not math.isfinite(t):
            raise ValueError("Expected finite timestamp")
        self.previous_t=float(t)
        self.previous_v=vec3(velocity_world)
        self.previous_q=normalize_quat(quat_wb)

    def measure(self,t,velocity_world,quat_wb,gravity_w=(0.,0.,-9.81)):
        if self.previous_t is None:raise ValueError("Initialize IMU with reset")
        dt=float(t)-self.previous_t
        if not math.isfinite(dt) or not 0<dt<=.1:
            raise ValueError("IMU timestamps must increase by <=0.1s")
        v=vec3(velocity_world)
        q=normalize_quat(quat_wb)
        g=vec3(gravity_w)
        # Average specific force over the observation interval expressed in
        # the mid-interval body frame; neglects lever arm, vibrations,
        # accelerometer axis misalignment and filtering latency.
        accel_w=(v-self.previous_v)/dt
        qmid=self.previous_q+np.sign(np.dot(self.previous_q,q))*q
        qmid=normalize_quat(qmid)
        body_force=quat_to_rot(qmid).T@(accel_w-g)
        omega=quat_body_delta(self.previous_q,q)/dt
        f=body_force+self.accel_bias+np.array([
            self.rng.gauss(0,self.accel_noise_std) for _ in range(3)])
        w=omega+self.gyro_bias+np.array([
            self.rng.gauss(0,self.gyro_noise_std) for _ in range(3)])
        self.previous_t=float(t)
        self.previous_v=v
        self.previous_q=q
        return IMUPacket(float(t),dt,tuple(map(float,f)),tuple(map(float,w)))
