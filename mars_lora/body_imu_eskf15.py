"""Research 15-error-state inertial navigation prototype.

Nominal state: p_W(3), v_W(3), q_WB(w,x,y,z), b_a(3), b_g(3).
Error state: [delta_p,delta_v,delta_theta_B,delta_ba,delta_bg].
Right-multiplicative attitude error: true R_WB = Rhat_WB Exp(delta_theta).
IMU inputs are body-frame *specific force* (includes -gravity at rest)
and 3-axis body angular velocity, NOT world-frame linear acceleration.

This is a filter research prototype, not certified flight navigation.
It assumes known initial attitude, known gravity and synchronized sensors.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
import numpy as np


def vec3(x):
    a=np.asarray(x,dtype=float)
    if a.shape!=(3,) or not np.all(np.isfinite(a)):
        raise ValueError("Expected a finite three-vector")
    return a.copy()


def skew(x):
    x=vec3(x)
    return np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])


def normalize_quat(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)):
        raise ValueError("Expected a finite wxyz quaternion")
    length=np.linalg.norm(q)
    if length<1.e-12:raise ValueError("Zero quaternion")
    return q/length


def qmul(a,b):
    w,x,y,z=normalize_quat(a)
    t,u,v,r=normalize_quat(b)
    return normalize_quat([w*t-x*u-y*v-z*r,
                          w*u+x*t+y*r-z*v,
                          w*v-x*r+y*t+z*u,
                          w*r+x*v-y*u+z*t])


def qconj(a):
    w,x,y,z=normalize_quat(a)
    return np.array([w,-x,-y,-z])


def exp_quat(rotvec):
    a=vec3(rotvec)
    mag=np.linalg.norm(a)
    if mag<1.e-8:
        # Exact first-order small-angle derivative, normalized below.
        return normalize_quat(np.r_[1.-mag*mag/8.,.5*a])
    return np.r_[math.cos(mag/2),a*math.sin(mag/2)/mag]


def quat_to_rot(q):
    w,x,y,z=normalize_quat(q)
    return np.array([
        [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def quat_body_delta(before,after):
    """Measured body rotation from two time-aligned world-from-body quats."""
    dq=qmul(qconj(before),after)
    if dq[0]<0:dq=-dq
    mag=np.linalg.norm(dq[1:])
    if mag<1.e-12:return np.zeros(3)
    angle=2*math.atan2(mag,dq[0])
    return dq[1:]/mag*angle


@dataclass
class FusionResult:
    accepted:bool
    nis:float
    gate:float
    reason:str


@dataclass
class InertialESKF15:
    position:np.ndarray
    velocity:np.ndarray=field(default_factory=lambda:np.zeros(3))
    quat_wb:np.ndarray=field(default_factory=lambda:np.array([1.,0.,0.,0.]))
    accel_bias:np.ndarray=field(default_factory=lambda:np.zeros(3))
    gyro_bias:np.ndarray=field(default_factory=lambda:np.zeros(3))
    P:np.ndarray=field(default_factory=lambda:np.diag(
        [.07**2]*3+[.08**2]*3+[math.radians(2)**2]*3+
        [.02**2]*3+[math.radians(.3)**2]*3))
    gravity_w:np.ndarray=field(default_factory=lambda:np.array([0.,0.,-9.81]))
    accel_noise_density:float=.04
    gyro_noise_density:float=.006
    accel_bias_rw_density:float=.002
    gyro_bias_rw_density:float=.0005
    accepted:int=0
    rejected:int=0
    time_s:float=0.
    last_nis:float=0.

    def __post_init__(self):
        self.position=vec3(self.position)
        self.velocity=vec3(self.velocity)
        self.quat_wb=normalize_quat(self.quat_wb)
        self.accel_bias=vec3(self.accel_bias)
        self.gyro_bias=vec3(self.gyro_bias)
        self.gravity_w=vec3(self.gravity_w)
        self.P=np.asarray(self.P,dtype=float).copy()
        if self.P.shape!=(15,15) or not np.all(np.isfinite(self.P)):
            raise ValueError("Expected finite 15x15 covariance")
        self.P=(self.P+self.P.T)*.5
        if np.linalg.eigvalsh(self.P).min()<=0:
            raise ValueError("Covariance must be strictly positive definite")
        noises=(self.accel_noise_density,self.gyro_noise_density,
                self.accel_bias_rw_density,self.gyro_bias_rw_density)
        if any(not math.isfinite(n) or n<=0 for n in noises):
            raise ValueError("Noise densities must be positive and finite")

    @classmethod
    def initial(cls,launch=(0.,0.,4.),quat_wb=(1.,0.,0.,0.),**kwargs):
        return cls(position=vec3(launch),quat_wb=normalize_quat(quat_wb),**kwargs)

    @property
    def sigma_position_rms_m(self):
        return float(math.sqrt(max(0,np.trace(self.P[:3,:3]))))

    def predict(self,specific_force_b,angular_rate_b,dt):
        if not math.isfinite(dt) or not 0<dt<=.1:
            raise ValueError("IMU time increment must be (0, 0.1] s")
        f=vec3(specific_force_b)-self.accel_bias
        omega=vec3(angular_rate_b)-self.gyro_bias
        # Midpoint world attitude makes the propagation independent of the
        # arbitrary quaternion sign and more accurate under rotation.
        Rmid=quat_to_rot(qmul(self.quat_wb,exp_quat(.5*omega*dt)))
        accel_w=Rmid@f+self.gravity_w
        self.position+=self.velocity*dt+.5*accel_w*dt*dt
        self.velocity+=accel_w*dt
        self.quat_wb=qmul(self.quat_wb,exp_quat(omega*dt))
        # Standard right-error linearization:
        # delta_v_dot = -R[f]x delta_theta -R delta_ba
        # delta_theta_dot = -[omega]x delta_theta -delta_bg.
        A=np.zeros((15,15))
        A[:3,3:6]=np.eye(3)
        A[3:6,6:9]=-Rmid@skew(f)
        A[3:6,9:12]=-Rmid
        A[6:9,6:9]=-skew(omega)
        A[6:9,12:15]=-np.eye(3)
        F=np.eye(15)+A*dt+.5*(A@A)*dt*dt
        G=np.zeros((15,12))
        G[3:6,0:3]=Rmid
        G[6:9,3:6]=np.eye(3)
        G[9:12,6:9]=np.eye(3)
        G[12:15,9:12]=np.eye(3)
        qc=np.diag([self.accel_noise_density**2]*3+
                   [self.gyro_noise_density**2]*3+
                   [self.accel_bias_rw_density**2]*3+
                   [self.gyro_bias_rw_density**2]*3)
        # First-order discretized continuous-time noise (O(dt)); estimator
        # performance still requires consistency testing under motion.
        self.P=F@self.P@F.T+(G@qc@G.T)*dt
        self.P=(self.P+self.P.T)*.5
        self.time_s+=dt

    def _update(self,observed,estimated,indices,std,gate):
        z=np.asarray(observed,dtype=float)
        y=np.asarray(estimated,dtype=float)
        if (z.ndim!=1 or z.shape!=y.shape or
                not np.all(np.isfinite(z)) or not np.all(np.isfinite(y))):
            raise ValueError("Invalid finite measurement")
        if not math.isfinite(std) or std<=0 or not math.isfinite(gate) or gate<=0:
            raise ValueError("Invalid measurement uncertainty/gate")
        H=np.zeros((z.size,15))
        for i,j in enumerate(indices):H[i,j]=1.
        R=np.eye(z.size)*std*std
        nu=z-y
        S=H@self.P@H.T+R
        nis=float(nu@np.linalg.solve(S,nu))
        self.last_nis=nis
        if not math.isfinite(nis) or nis>gate:
            self.rejected+=1
            return FusionResult(False,nis,gate,"innovation_gate")
        K=np.linalg.solve(S,H@self.P).T
        dx=K@nu
        self.position+=dx[:3]
        self.velocity+=dx[3:6]
        self.quat_wb=qmul(self.quat_wb,exp_quat(dx[6:9]))
        self.accel_bias+=dx[9:12]
        self.gyro_bias+=dx[12:15]
        ident=np.eye(15)
        IK=ident-K@H
        self.P=IK@self.P@IK.T+K@R@K.T
        # Right-multiplicative injection reset Jacobian.
        reset=np.eye(15)
        reset[6:9,6:9]-=.5*skew(dx[6:9])
        self.P=reset@self.P@reset.T
        self.P=(self.P+self.P.T)*.5
        self.accepted+=1
        return FusionResult(True,nis,gate,"accepted")

    def correct_xyz(self,xyz,std_m=.2,gate=11.345):
        return self._update(vec3(xyz),self.position,(0,1,2),std_m,gate)

    def correct_xy(self,xy,std_m=.25,gate=9.210):
        xy=np.asarray(xy,dtype=float)
        return self._update(xy,self.position[:2],(0,1),std_m,gate)

    def correct_altitude(self,z,std_m=.06,gate=6.635):
        return self._update(np.array([z],dtype=float),
                            self.position[2:3],(2,),std_m,gate)

    def correct_horizontal_flow(self,vx,vy,std_m=.045,gate=9.210):
        return self._update(np.array([vx,vy],dtype=float),
                            self.velocity[:2],(3,4),std_m,gate)

    def position_nees(self,truth_position):
        """For offline metrics only. Never call to alter state/covariance."""
        err=vec3(truth_position)-self.position
        return float(err@np.linalg.solve(self.P[:3,:3],err))
