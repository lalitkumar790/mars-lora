"""Research-only translational error-state Kalman filter for a world-frame IMU surrogate.

State: 3D position, 3D velocity, 3D *accelerometer* bias (9 error states).
This intentionally matches the EXISTING MuJoCo surrogate, whose acceleration
is already world-frame and gravity-compensated. It is NOT a full 15-state
orientation ESKF, a real body-frame IMU model, or an unknown-map SLAM system.

All scan-match positions are measurements made against the supplied prior map;
MuJoCo truth must enter this filter only as *synthetic sensor measurements*.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
import numpy as np

@dataclass
class UpdateResult:
    accepted:bool
    nis:float
    gate:float
    reason:str

@dataclass
class TranslationalESKF:
    position:np.ndarray
    velocity:np.ndarray=field(default_factory=lambda:np.zeros(3))
    bias:np.ndarray=field(default_factory=lambda:np.zeros(3))
    P:np.ndarray=field(default_factory=lambda:np.diag([.06**2]*3+[.08**2]*3+[.015**2]*3))
    accel_noise_sigma_m_s2:float=.025
    bias_rw_sigma_m_s2_sqrt_s:float=.002
    accepted:int=0
    rejected:int=0
    last_nis:float=0.
    time_s:float=0.

    def __post_init__(self):
        self.position=self._vec(self.position,3)
        self.velocity=self._vec(self.velocity,3)
        self.bias=self._vec(self.bias,3)
        self.P=np.array(self.P,dtype=float,copy=True)
        if self.P.shape!=(9,9) or not np.all(np.isfinite(self.P)):
            raise ValueError("Expected finite 9x9 covariance")
        self.P=(self.P+self.P.T)*.5
        if np.linalg.eigvalsh(self.P).min()<=0:
            raise ValueError("Initial covariance must be positive definite")
        if not (self.accel_noise_sigma_m_s2>0 and self.bias_rw_sigma_m_s2_sqrt_s>0):
            raise ValueError("Process standard deviations must be positive")

    @staticmethod
    def _vec(v,n):
        x=np.asarray(v,dtype=float)
        if x.shape!=(n,) or not np.all(np.isfinite(x)):
            raise ValueError(f"Expected {n} finite measurements")
        return x.copy()

    @classmethod
    def initial(cls,launch=(0.,0.,4.),**kwargs):
        return cls(cls._vec(launch,3),**kwargs)

    @property
    def position_sigma_axes_m(self):
        return np.sqrt(np.maximum(0,np.diag(self.P)[:3]))

    @property
    def position_rms_sigma_m(self):
        return float(np.sqrt(max(0,np.trace(self.P[:3,:3]))))

    def predict(self,world_accel_measured,dt):
        if not np.isfinite(dt) or dt<=0 or dt>.25:
            raise ValueError("IMU delta time must be 0 < dt <= 0.25s")
        accel=self._vec(world_accel_measured,3)-self.bias
        self.position+=self.velocity*dt+.5*accel*dt*dt
        self.velocity+=accel*dt
        F=np.eye(9)
        F[:3,3:6]=np.eye(3)*dt
        F[:3,6:9]=-np.eye(3)*(.5*dt*dt)
        F[3:6,6:9]=-np.eye(3)*dt
        # Continuous white input-acceleration spectral density approximation.
        G=np.zeros((9,3))
        G[:3,:]=np.eye(3)*(.5*dt*dt)
        G[3:6,:]=np.eye(3)*dt
        Qacc=G@G.T*(self.accel_noise_sigma_m_s2**2)
        Qbias=np.zeros((9,9))
        Qbias[6:9,6:9]=np.eye(3)*self.bias_rw_sigma_m_s2_sqrt_s**2*dt
        self.P=F@self.P@F.T+Qacc+Qbias
        self.P=(self.P+self.P.T)*.5
        self.time_s+=dt

    def _correct(self,observed,indices,std_m,nis_limit):
        indices=tuple(indices)
        z=self._vec(observed,len(indices))
        if not np.isfinite(std_m) or std_m<=0 or not np.isfinite(nis_limit) or nis_limit<=0:
            raise ValueError("Measurement std and NIS gate must be finite positive")
        x=np.r_[self.position,self.velocity,self.bias]
        H=np.zeros((len(indices),9))
        for row,index in enumerate(indices):
            H[row,index]=1.
        R=np.eye(len(indices))*std_m**2
        innovation=z-H@x
        S=H@self.P@H.T+R
        nis=float(innovation@np.linalg.solve(S,innovation))
        self.last_nis=nis
        if not math.isfinite(nis) or nis>nis_limit:
            self.rejected+=1
            return UpdateResult(False,nis,nis_limit,"innovation_gate")
        K=np.linalg.solve(S,self.P@H.T.T if False else (H@self.P)).T
        dx=K@innovation
        self.position+=dx[:3]
        self.velocity+=dx[3:6]
        self.bias+=dx[6:9]
        I=np.eye(9)
        IK=I-K@H
        self.P=IK@self.P@IK.T+K@R@K.T
        self.P=(self.P+self.P.T)*.5
        self.accepted+=1
        return UpdateResult(True,nis,nis_limit,"accepted")

    def update_position(self,xyz,std_m=.15,nis_limit=11.345):
        # 99% chi-squared gate for three scalar measurements.
        return self._correct(xyz,(0,1,2),std_m,nis_limit)

    def update_altitude(self,z,std_m=.06,nis_limit=6.635):
        # 99% chi-squared gate with one degree of freedom.
        return self._correct((z,),(2,),std_m,nis_limit)

    def update_horizontal_flow(self,vx,vy,std_m=.045,nis_limit=9.210):
        # 99% chi-squared gate with two degrees of freedom.
        return self._correct((vx,vy),(3,4),std_m,nis_limit)

    def update_planar_map_position(self,xy,std_m=.20,nis_limit=9.210):
        return self._correct(xy,(0,1),std_m,nis_limit)

    def position_nees(self,truth_position):
        """Research-only metric. Ground truth must not feed filter updates."""
        e=self._vec(truth_position,3)-self.position
        return float(e@np.linalg.solve(self.P[:3,:3],e))
