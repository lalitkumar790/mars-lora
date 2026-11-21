"""Full mission validation for browser MJCF airframes using actual MuJoCo physics.

This mirrors the browser controller's principal control law. It is NOT a
bit-for-bit browser execution nor a proof of flight safety.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import random
from pathlib import Path
import mujoco
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mars_lora.urban_scene import add_city_mjcf
from mars_lora.rf_astar import SOLIDS, INDUSTRIAL, signed_clearance as full_city_clearance
from mars_lora.urban_scene import city, CITY_SEED
from mars_lora.lidar_map_matching import observe_scan,match_scan
from mars_lora.lidar_imu3d import InertialMapEstimator, simulate_scan

ROUTE=[(0,0,7),(2,0,7),(2,8,7),(-7,8,7),(-7,-7,7),(0,0,7),(0,0,2.5)]
OBSTACLES=[
 ("east",5,4,1.4,1.1,1.4,1.4),("west",-5,5,1.9,1.25,1.1,1.9),
 ("south",5,-4,1.6,1.4,1,1.6),("container",-3,-4,.6,2,.7,.6),
 ("tower",1,8,2.5,.24,.24,2.5),("pylon",-8,0,2.7,.32,.32,2.7),
 ("beam",-5,0,5.45,3.3,.24,.23)
]
def signed_clearance(p,radius):
    best=1e6
    for _,x,y,z,hx,hy,hz in OBSTACLES:
        d=[abs(p[0]-x)-hx,abs(p[1]-y)-hy,abs(p[2]-z)-hz]
        outside=math.sqrt(sum(max(v,0)**2 for v in d))
        dist=-min(-v for v in d) if all(v<=0 for v in d) else outside
        best=min(best,dist-radius)
    return best
# Conservative spatial broadphase: retain every city/industrial solid that
# could be <=3 m from a point in this XY grid tile. Exact 3D clearance is
# calculated only for those candidate solids; values greater than 3 m are
# intentionally capped at 3 m. This keeps 250-Hz physics audits tractable.
CLEARANCE_CAP=3.
CLEARANCE_CELL_M=6.
def make_clearance_grid(solids=SOLIDS):
    from collections import defaultdict
    cells=defaultdict(list)
    for solid in solids:
        cx,cy,cz,hx,hy,hz=solid
        extent=CLEARANCE_CAP+.5
        xmin=math.floor((cx-hx-extent)/CLEARANCE_CELL_M)
        xmax=math.floor((cx+hx+extent)/CLEARANCE_CELL_M)
        ymin=math.floor((cy-hy-extent)/CLEARANCE_CELL_M)
        ymax=math.floor((cy+hy+extent)/CLEARANCE_CELL_M)
        for ix in range(xmin,xmax+1):
            for iy in range(ymin,ymax+1):
                cells[(ix,iy)].append(solid)
    return dict(cells)

CLEARANCE_GRID=make_clearance_grid()
def fast_city_clearance(p,radius=0.,grid=None):
    from mars_lora.rf_astar import signed_clearance
    key=(math.floor(p[0]/CLEARANCE_CELL_M),math.floor(p[1]/CLEARANCE_CELL_M))
    neighbors=(CLEARANCE_GRID if grid is None else grid).get(key,())
    if not neighbors:return CLEARANCE_CAP
    return min(CLEARANCE_CAP,signed_clearance(p,neighbors)-radius)

def clamp(x,a,b):
    return max(a,min(b,x))
def quat_to_rpy(q):
    w,x,y,z=q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(clamp(2*(w*y-z*x),-1,1)),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def run(airframe,duration=240,wind=.25,out=None,route=None,require_completion=False,
        controller_mode='legacy',wind_phase=0.,city_seed=CITY_SEED,
        localization_drift_m_sqrt_s=0.,localization_noise_sigma_m=0.,
        localization_seed=0,localization_mode='drift',dynamic_replanning=False,
        lidar_blackout_window=None,replan_stopping_envelope=False,
        altitude_slew_m_s=None,trajectory_speed_limits=None,
        trajectory_reference=None,online_risk_monitor=False,eskf_shadow=False,
        eskf15_shadow=False,pose6_shadow=False):
    if controller_mode not in ('legacy','high_authority','integral_guard','barrier_guard','barrier_ttc'):
        raise ValueError('Unsupported controller mode')
    if localization_mode not in ('drift','lidar_match','lidar_imu3d','lidar_imu3d_hybrid'):
        raise ValueError('Unknown navigation localization mode')
    if pose6_shadow and not eskf15_shadow:
        raise ValueError('6D observability requires 15-state shadow observer')
    if eskf15_shadow and localization_mode!='lidar_imu3d_hybrid':
        raise ValueError('15-state shadow requires original hybrid sensor stream')
    if eskf_shadow and localization_mode!='lidar_imu3d_hybrid':
        raise ValueError('ESKF shadow comparison requires native hybrid sensor stream')
    if altitude_slew_m_s is not None and (not math.isfinite(altitude_slew_m_s) or altitude_slew_m_s<=0):
        raise ValueError('Altitude slew rate must be positive and finite')
    if localization_drift_m_sqrt_s<0 or localization_noise_sigma_m<0:
        raise ValueError('Localization perturbation standard deviations must be nonnegative')
    waypoints=list(ROUTE if route is None else route)
    if len(waypoints)<2:raise ValueError('A flight requires at least 2 waypoints')
    if trajectory_reference is not None and (
            dynamic_replanning or trajectory_speed_limits is not None or
            altitude_slew_m_s is not None or
            len(trajectory_reference.samples)<2):
        raise ValueError('Timed reference is exclusive with replanning, slew and waypoint-speed tracking')

    if trajectory_speed_limits is not None and (len(trajectory_speed_limits)!=len(waypoints) or
        any(not math.isfinite(v) or v<0 for v in trajectory_speed_limits)):
        raise ValueError('Trajectory speed limits must be finite and align with waypoints')
    root=Path(__file__).resolve().parents[1]
    xml=(root/"public"/("crazyflie.xml" if airframe=="crazyflie" else "quad.xml")).read_text()
    model=mujoco.MjModel.from_xml_string(add_city_mjcf(xml,seed=city_seed))
    scene_solids=INDUSTRIAL+tuple(city(city_seed))
    scene_grid=(CLEARANCE_GRID if city_seed==CITY_SEED else make_clearance_grid(scene_solids))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    mass=.027 if airframe=="crazyflie" else 1.325
    armx,army=(.046,.046) if airframe=="crazyflie" else (.14,.18)
    moment_scale=2.3951e-5/.019 if airframe=="crazyflie" else 1.
    max_thrust=.15 if airframe=="crazyflie" else 13.
    desired=[0.,0.,.5]
    rotor=[mass*9.81/4]*4
    idx=0;contacts=0;minimum=1e6;max_tilt=0;max_error=0;last_controller=-1
    x_integral=y_integral=0.0; safety_holds=0;max_lateral_error=0.
    records=[];cross_track=[]
    # This models pose-ESTIMATE perturbations, not a functioning SLAM solver.
    # Orientation and velocity remain MuJoCo truth/IMU proxies: optimistic.
    localization_rng=random.Random(localization_seed)
    lidar_rng=random.Random(localization_seed+991)
    localization_bias=[0.,0.]
    map_correction=[0.,0.]
    inertial=None
    # Fixed per-flight accelerometer offset, in m/s^2. The previous prototype
    # mistakenly integrated position-random-walk bias directly as m/s^2,
    # which blew up into kilometer-scale pose errors. Preserve its V1 results.
    imu_bias_rng=random.Random(localization_seed+1997)
    imu_accel_bias=[imu_bias_rng.gauss(0.,.003) for _ in range(3)]
    imu_last_velocity=[float(x) for x in data.qvel[:3]]
    map_replans=map_replan_failures=localization_lost_events=0
    first_contact=None;first_clearance_violation=None;minimum_clearance_event=None
    risk_alarm_first=None;risk_alarm_count=0;risk_monitor_checks=0
    loss_of_confidence_first=None;loss_of_confidence_samples=0
    stopping_envelope_rejections=0
    reference_start_t=None;reference_index=0;reference_settle_s=None
    accepted_replan_events=[];rejected_replan_events=[]
    last_replan_time=-100.
    replan_anchor_xy=None
    if localization_mode in ('lidar_imu3d','lidar_imu3d_hybrid'):
        inertial=InertialMapEstimator.initial(tuple(float(x) for x in data.qpos[:3]))
        inertial.process_sigma_m_sqrt_s=max(.012,localization_drift_m_sqrt_s)
    shadow=None
    shadow_error_squared=0.;shadow_error_max=0.;shadow_samples=0
    shadow_precontact_squared=0.;shadow_precontact_count=0
    legacy_precontact_squared=0.;legacy_precontact_count=0
    shadow_nees_99_inside=0;shadow_scan_accepted=0;shadow_scan_rejected=0
    shadow_planar_accepted=0;shadow_planar_rejected=0
    if eskf_shadow:
        from mars_lora.lidar_imu_eskf import TranslationalESKF
        shadow=TranslationalESKF.initial(tuple(float(x) for x in data.qpos[:3]))
    body15=None;body_imu_sensor=None
    body15_errsq=0.;body15_precontact_sq=0.;body15_samples=0.;body15_precontact_samples=0
    body15_coverage99=0;body15_lidar3d_accept=body15_lidar3d_reject=0
    body15_first_error_1m=None;body15_first_error_5m=None
    body15_precontact_attitude_squared=0.;body15_precontact_attitude_count=0
    body15_planar_accept=body15_planar_reject=0;body15_max_error=0.
    pose6_rng=None;pose6_rows=[];pose6_rank_hist={i:0 for i in range(7)}
    pose6_full_rank=0;pose6_partial=0
    if pose6_shadow:
        pose6_rng=random.Random(localization_seed+901733)

    observable_scan_count=0;unobservable_scan_count=0
    observable_ranks={0:0,1:0,2:0,3:0}
    last_informative_scan_t=None;longest_observation_gap_s=0.
    first_unobservable_scan=None;first_high_residual_scan=None
    scan_observability_rows=[]

    if eskf15_shadow:
        from mars_lora.body_imu_eskf15 import InertialESKF15
        from mars_lora.body_imu_surrogate import BodyIMUSurrogate
        body15=InertialESKF15.initial(
            launch=tuple(float(x) for x in data.qpos[:3]),
            quat_wb=tuple(float(x) for x in data.qpos[3:7]))
        body_imu_sensor=BodyIMUSurrogate(seed=localization_seed+33071)
        body_imu_sensor.reset(float(data.time),
            tuple(float(x) for x in data.qvel[:3]),
            tuple(float(x) for x in data.qpos[3:7]))
    lidar_accepted=lidar_rejected=0
    next_lidar_time=1.0
    localization_sq_sum=0.;localization_max_error=0.;localization_updates=0
    dt=model.opt.timestep
    steps=round(duration/dt)
    for k in range(steps):
        t=float(data.time)
        p=data.qpos;v=data.qvel
        if k%5==0:
            roll,pitch,yaw=quat_to_rpy(p[3:7])
            localization_dt=5*dt
            if localization_drift_m_sqrt_s:
                for axis in (0,1):
                    localization_bias[axis]+=localization_drift_m_sqrt_s*math.sqrt(localization_dt)*localization_rng.gauss(0.,1.)
            if localization_mode in ('lidar_imu3d','lidar_imu3d_hybrid'):
                # Synthetic IMU packet is generated from *finite differenced*
                # native velocity plus independent seeded sensor perturbation.
                # The estimator/controller only consume the noisy packet.
                # No ground-truth XY or body velocity enters this controller.
                measured_accel=[
                    (float(v[i])-imu_last_velocity[i])/localization_dt
                    +imu_accel_bias[i] for i in range(3)]
                measured_accel=[clamp(a+localization_rng.gauss(0.,.025),-6.,6.)
                                for a in measured_accel]
                for axis in range(3):imu_last_velocity[axis]=float(v[axis])
                inertial.predict(measured_accel,
                                float(v[5])+localization_rng.gauss(0.,.003),
                                localization_dt)
                if shadow is not None:
                    shadow.predict(measured_accel,localization_dt)
                if body15 is not None and t>body_imu_sensor.previous_t+1.e-9:
                    packet=body_imu_sensor.measure(
                        t,tuple(float(a) for a in v[:3]),
                        tuple(float(a) for a in p[3:7]))
                    body15.predict(packet.specific_force_b,
                                   packet.angular_rate_b,packet.dt_s)
                if localization_mode=='lidar_imu3d_hybrid':
                    # Actual native truth ONLY generates noisy surrogate
                    # barometer and ground-optical-flow sensor observations.
                    z_obs=float(p[2])+localization_rng.gauss(0.,.06)
                    inertial.correct_altitude(z_obs)
                    if shadow is not None:
                        shadow.update_altitude(z_obs,std_m=.06)
                    if body15 is not None:
                        body15.correct_altitude(z_obs,std_m=.06)
                    if p[2]<12.:  # flow degenerates above texture/range limit
                        flowx=float(v[0])+localization_rng.gauss(0.,.045)
                        flowy=float(v[1])+localization_rng.gauss(0.,.045)
                        inertial.correct_horizontal_flow(flowx,flowy)
                        if shadow is not None:
                            shadow.update_horizontal_flow(flowx,flowy,std_m=.045)
                        if body15 is not None:
                            body15.correct_horizontal_flow(flowx,flowy,std_m=.045)
                if t>=next_lidar_time:
                    blackout=(lidar_blackout_window is not None and
                              lidar_blackout_window[0]<=t<=lidar_blackout_window[1])
                    if blackout:
                        inertial.sigma_m=min(5.,inertial.sigma_m+.028)
                        lidar_rejected+=1
                    else:
                        reading=simulate_scan(tuple(float(x) for x in p[:3]),
                                              yaw,scene_solids,lidar_rng,
                                              noise=.06,dropout=.03)
                        if shadow is not None:
                            from mars_lora.lidar_imu3d import register_scan
                            shadow_candidate,shadow_fit,shadow_score,shadow_hits=register_scan(
                                tuple(shadow.position),inertial.yaw,reading,scene_solids)
                            if shadow_fit:
                                check=shadow.update_position(shadow_candidate,std_m=.20)
                                shadow_scan_accepted+=int(check.accepted)
                                shadow_scan_rejected+=int(not check.accepted)
                            else:
                                shadow_scan_rejected+=1
                        if pose6_shadow:
                            # Independent body-fixed sensor packet, not
                            # the legacy yaw-only range measurement.
                            from mars_lora.lidar_pose6_observability import (
                                simulate_body_scan,assess_pose6)
                            scan6=simulate_body_scan(
                                tuple(float(x) for x in p[:3]),
                                tuple(float(x) for x in p[3:7]),
                                scene_solids,pose6_rng,noise=.06,dropout=.03)
                            assessment=assess_pose6(
                                tuple(body15.position),tuple(body15.quat_wb),
                                scan6,scene_solids)
                            pose6_rank_hist[assessment.rank6]+=1
                            pose6_full_rank+=int(assessment.locally_full_rank)
                            pose6_partial+=int(not assessment.locally_full_rank)
                            pose6_rows.append({
                                "t_s":round(float(t),3),
                                "rank6":int(assessment.rank6),
                                "rank_xyz":int(assessment.translation_rank),
                                "rank_rotation":int(assessment.rotation_rank),
                                "hits":int(assessment.hits),
                                "usable_hits":int(assessment.usable),
                                "fit_rmse_m":(round(float(assessment.residual_rmse_m),4)
                                    if math.isfinite(assessment.residual_rmse_m) else None),
                                "full_rank":bool(assessment.locally_full_rank)})
                        if body15 is not None:
                            from mars_lora.body_imu_eskf15 import quat_to_rot
                            from mars_lora.lidar_imu3d import register_scan
                            rot=quat_to_rot(body15.quat_wb)
                            yaw15=math.atan2(float(rot[1,0]),float(rot[0,0]))
                            pos15,valid15,score15,hits15=register_scan(
                                tuple(body15.position),yaw15,reading,scene_solids)
                            # Diagnostic quality must be computed from the
                            # estimated pose before applying a correction.
                            from mars_lora.lidar_observability import assess_scan_observability
                            observe=assess_scan_observability(
                                tuple(body15.position),yaw15,reading,scene_solids)
                            observable_ranks[observe.geometric_rank]+=1
                            if observe.informative:
                                observable_scan_count+=1
                                last_informative_scan_t=t
                            else:
                                unobservable_scan_count+=1
                                if first_unobservable_scan is None:
                                    first_unobservable_scan=round(float(t),3)
                            if (math.isfinite(observe.rms_range_residual_m) and
                                observe.rms_range_residual_m>.30 and
                                first_high_residual_scan is None):
                                first_high_residual_scan=round(float(t),3)
                            scan_observability_rows.append({
                                "t_s":round(float(t),3),
                                "hits":observe.valid_hits,
                                "usable":observe.used_hits,
                                "rank":observe.geometric_rank,
                                "info_min":round(observe.smallest_info_eigenvalue,4),
                                "rms_range_residual_m":(
                                    round(observe.rms_range_residual_m,4)
                                    if math.isfinite(observe.rms_range_residual_m)
                                    else None),
                                "informative":observe.informative,
                                "map_fit_accepted":bool(valid15)})
                            if valid15:
                                observation=body15.correct_xyz(pos15,std_m=.20)
                                body15_lidar3d_accept+=int(observation.accepted)
                                body15_lidar3d_reject+=int(not observation.accepted)
                            else:body15_lidar3d_reject+=1
                        accepted,scan_score,hits=inertial.correct(reading,scene_solids)
                        if accepted:lidar_accepted+=1
                        else:
                            lidar_rejected+=1
                            if localization_mode=='lidar_imu3d_hybrid':
                                # Independent planar subset provides bearings
                                # for ground-level known-map correction when
                                # tilted 3D rays are uninformative.
                                horizontal=observe_scan(
                                    (float(p[0]),float(p[1])),yaw,float(p[2]),
                                    scene_solids,lidar_rng,noise_sigma=.06)
                                prior_xy=(inertial.position[0],inertial.position[1])
                                corrected,ok,_=match_scan(
                                    prior_xy,inertial.yaw,inertial.position[2],
                                    horizontal,scene_solids)
                                if shadow is not None:
                                    # Reuse EXACTLY the planar scan already
                                    # created for legacy's 3D fallback. No
                                    # additional random calls or truth enters.
                                    shadow_xy,shadow_ok,_=match_scan(
                                        tuple(shadow.position[:2]),inertial.yaw,
                                        float(shadow.position[2]),horizontal,scene_solids)
                                    if shadow_ok:
                                        update=shadow.update_planar_map_position(
                                            shadow_xy,std_m=.25)
                                        shadow_planar_accepted+=int(update.accepted)
                                        shadow_planar_rejected+=int(not update.accepted)
                                    else:
                                        shadow_planar_rejected+=1
                                if body15 is not None:
                                    rot=quat_to_rot(body15.quat_wb)
                                    yaw15=math.atan2(float(rot[1,0]),float(rot[0,0]))
                                    xy15,valid15,_=match_scan(
                                        tuple(body15.position[:2]),yaw15,
                                        float(body15.position[2]),horizontal,scene_solids)
                                    if valid15:
                                        outcome=body15.correct_xy(xy15,std_m=.25)
                                        body15_planar_accept+=int(outcome.accepted)
                                        body15_planar_reject+=int(not outcome.accepted)
                                    else:body15_planar_reject+=1
                                inertial.correct_planar_lidar(corrected,ok)
                                if ok:lidar_accepted+=1
                    next_lidar_time+=2.
                raw_xy=(inertial.position[0],inertial.position[1])
                nav_xyz=tuple(inertial.position)
                nav_velocity=tuple(inertial.velocity)
                nav_yaw=inertial.yaw
            else:
                raw_xy=tuple(float(p[axis])+localization_bias[axis]+(
                    localization_noise_sigma_m*localization_rng.gauss(0.,1.)
                    if localization_noise_sigma_m else 0.) for axis in (0,1))
            if localization_mode=='lidar_match' and t>=next_lidar_time:
                blackout=(lidar_blackout_window is not None and
                          lidar_blackout_window[0]<=t<=lidar_blackout_window[1])
                if blackout:
                    lidar_rejected+=1
                else:
                    scan=observe_scan((float(p[0]),float(p[1])),yaw,float(p[2]),
                                      scene_solids,lidar_rng,noise_sigma=.06)
                    prior=(raw_xy[0]+map_correction[0],raw_xy[1]+map_correction[1])
                    corrected,accepted,scan_score=match_scan(
                        prior,yaw,float(p[2]),scan,scene_solids)
                    if accepted:
                        map_correction[0]=corrected[0]-raw_xy[0]
                        map_correction[1]=corrected[1]-raw_xy[1]
                        lidar_accepted+=1
                    else:lidar_rejected+=1
                next_lidar_time+=2.0
            nav_xy=(raw_xy if localization_mode in ('lidar_imu3d','lidar_imu3d_hybrid') else
                    (raw_xy[0]+map_correction[0],raw_xy[1]+map_correction[1]))
            if localization_mode not in ('lidar_imu3d','lidar_imu3d_hybrid'):
                nav_xyz=(nav_xy[0],nav_xy[1],float(p[2]))
                nav_velocity=tuple(float(x) for x in v[:3])
                nav_yaw=yaw
            navigation_error=(math.dist(nav_xyz,p[:3]) if localization_mode in ('lidar_imu3d','lidar_imu3d_hybrid')
                              else math.hypot(nav_xy[0]-p[0],nav_xy[1]-p[1]))
            if shadow is not None:
                # Offline truth is used for scoring only; shadow never
                # drives control inputs, the baseline estimator or replans.
                distance=math.dist(shadow.position,p[:3])
                shadow_error_squared+=distance*distance
                if first_contact is None:
                    shadow_precontact_squared+=distance*distance
                    shadow_precontact_count+=1
                shadow_error_max=max(shadow_error_max,distance)
                shadow_samples+=1
                shadow_nees_99_inside+=int(shadow.position_nees(p[:3])<=11.345)
            if first_contact is None:
                legacy_precontact_squared+=navigation_error**2
                legacy_precontact_count+=1
            if body15 is not None:
                # Count time since last geometrically informative scan;
                # do not treat a barometer/flow correction as 3D observability.
                last=last_informative_scan_t
                if last is not None:
                    longest_observation_gap_s=max(
                        longest_observation_gap_s,float(t)-last)
                err15=math.dist(body15.position,p[:3])
                body15_samples+=1
                body15_errsq+=err15*err15
                body15_max_error=max(body15_max_error,err15)
                if err15>=1. and body15_first_error_1m is None:
                    body15_first_error_1m={"t_s":round(float(t),3),
                        "error_m":round(float(err15),3),
                        "sigma_position_rms_m":round(float(body15.sigma_position_rms_m),3)}
                if err15>=5. and body15_first_error_5m is None:
                    body15_first_error_5m={"t_s":round(float(t),3),
                        "error_m":round(float(err15),3),
                        "sigma_position_rms_m":round(float(body15.sigma_position_rms_m),3)}
                body15_coverage99+=int(body15.position_nees(p[:3])<=11.345)
                if first_contact is None:
                    body15_precontact_samples+=1
                    body15_precontact_sq+=err15*err15
                    from mars_lora.body_imu_eskf15 import quat_body_delta
                    attitude_delta=quat_body_delta(
                        tuple(float(a) for a in body15.quat_wb),
                        tuple(float(a) for a in p[3:7]))
                    angle=math.sqrt(sum(float(a)*float(a) for a in attitude_delta))
                    body15_precontact_attitude_squared+=angle*angle
                    body15_precontact_attitude_count+=1
            localization_sq_sum+=navigation_error**2
            localization_updates+=1
            localization_max_error=max(localization_max_error,navigation_error)
            if online_risk_monitor:
                from mars_lora.online_risk_monitor import predict_clearance_alarm
                sigma=(inertial.sigma_m if inertial is not None else
                       max(0.,localization_noise_sigma_m))
                radius=.08 if airframe=='crazyflie' else .32
                alarm=predict_clearance_alarm(
                    nav_xyz,nav_velocity,sigma,
                    lambda q:fast_city_clearance(q,radius,grid=scene_grid))
                risk_monitor_checks+=1
                # Independent of estimated map proximity: a sensor model
                # that cannot support a small pose-error bound must not be
                # treated as a valid geometric safety supervisor. This is
                # an experimental warning threshold, not calibrated NEES.
                if sigma>=.50:
                    loss_of_confidence_samples+=1
                    if loss_of_confidence_first is None:
                        loss_of_confidence_first={
                           "t_s":round(float(t),3),
                           "estimated_sigma_m":round(float(sigma),3),
                           "estimated_xyz":[round(float(v),3) for v in nav_xyz]}

                if alarm.warning:
                    risk_alarm_count+=1
                    if risk_alarm_first is None:
                        risk_alarm_first={
                            "t_s":round(float(t),3),
                            "reason":alarm.reason,
                            "predicted_margin_m":round(float(alarm.predicted_margin_m),3),
                            "stopping_distance_m":round(float(alarm.stopping_distance_m),3),
                            "estimated_sigma_m":round(float(sigma),3),
                            "estimated_xyz":[round(float(a),3) for a in nav_xyz]}

            target=waypoints[idx]
            if localization_mode in ('lidar_imu3d','lidar_imu3d_hybrid') and dynamic_replanning and (
                inertial.sigma_m>.20 and t-last_replan_time>24. and
                map_replans<2 and idx<len(waypoints)-1):
                # Dynamic uncertainty-triggered map-known A*: the estimator
                # drives start XYZ; truth is NEVER used for replanning.
                from mars_lora.rf_astar import plan
                try:
                    radius=(.08 if airframe=='crazyflie' else .32)
                    # Replanning is only legal if the estimator's *current*
                    # position has room for both uncertainty and braking.
                    # Otherwise preserve the previous verified route.
                    current_clearance=fast_city_clearance(nav_xyz,radius,grid=scene_grid)
                    if current_clearance<1.8:
                        raise ValueError('Replan deferred: no safe maneuver space')
                    updated=plan('shortest',solids=scene_solids,
                        start_xyz=tuple(nav_xyz),goal_xyz=tuple(waypoints[-1]),
                        radius=radius,margin=1.3,
                        localization_sigma_m=min(.45,inertial.sigma_m),
                        max_nodes=16000)
                    # Safe splice: the continuous transition from estimated
                    # current pose to the snapped A* grid origin is mandatory.
                    # V3 incorrectly discarded that first grid node and
                    # jumped toward node 2, cutting across nearby structures.
                    from mars_lora.rf_astar import path_clearance
                    radius=(.08 if airframe=='crazyflie' else .32)
                    transition=[tuple(nav_xyz),*updated.points]
                    required_margin=1.3+3*min(.45,inertial.sigma_m)
                    if (math.dist(transition[0],transition[1])>2.8 or
                        path_clearance(transition,radius=radius,
                                       margin=required_margin,
                                       solids=scene_solids)<required_margin-1.e-6):
                        raise ValueError('Unsafe replan splice')
                    if replan_stopping_envelope:
                        from mars_lora.replan_envelope import check_handoff
                        check=check_handoff(
                            nav_xyz,nav_velocity,updated.points,scene_solids,
                            vehicle_radius_m=radius,
                            localization_sigma_m=inertial.sigma_m,
                            required_clearance_m=.25,
                            braking_accel_m_s2=.8,
                            actuator_delay_s=.2,
                            wind_accel_bound_m_s2=.15)
                        if not check.accepted:
                            stopping_envelope_rejections+=1
                            raise ValueError('Unsafe stopping-envelope handoff: '+check.reason)
                    # Do not switch trajectories during significant lateral
                    # travel; an abrupt turn at speed creates a real contact.
                    if math.hypot(nav_velocity[0],nav_velocity[1])>.45:
                        raise ValueError('Replan handoff speed too high')
                    waypoints=transition[1:]
                    if replan_stopping_envelope:
                        # Track from the exact estimated pose validated by
                        # the stopping-envelope splice, not launch origin.
                        replan_anchor_xy=tuple(nav_xyz[:2])
                    idx=0;map_replans+=1
                    accepted_replan_events.append({"t_s":round(float(t),3),
                        "estimated_xyz":[round(float(a),3) for a in nav_xyz],
                        "uncertainty_sigma_m":round(float(inertial.sigma_m),3),
                        "speed_m_s":round(math.sqrt(sum(a*a for a in nav_velocity)),3)})
                    target=waypoints[idx]
                except (RuntimeError,ValueError,AssertionError) as error:
                    map_replan_failures+=1
                    rejected_replan_events.append({"t_s":round(float(t),3),
                        "reason":str(error),"sigma_m":round(float(inertial.sigma_m),3)})
                last_replan_time=t
            delta=[target[i]-nav_xyz[i] for i in range(3)]
            speed3=math.sqrt(sum(nav_velocity[i]**2 for i in range(3)))
            if trajectory_reference is None and math.sqrt(sum(x*x for x in delta))<.5 and speed3<.65 and idx<len(waypoints)-1:
                idx+=1
                if controller_mode=='integral_guard':
                    x_integral=y_integral=0.
                target=waypoints[idx]
            # Match browser lookahead tracking: project current XY onto the
            # segment, advance by 0.9m, and brake at each corner.
            ax0,ay0=((replan_anchor_xy if replan_anchor_xy is not None else (0,0))
                     if idx==0 else waypoints[idx-1][:2])
            dx,dy=target[0]-ax0,target[1]-ay0
            L=math.hypot(dx,dy)
            if L>1e-9:
                ux,uy=dx/L,dy/L
                along=clamp((nav_xy[0]-ax0)*ux+(nav_xy[1]-ay0)*uy,0,L)
                lookahead=1.5 if controller_mode in ('high_authority','barrier_guard','barrier_ttc') else (1.1 if controller_mode=='integral_guard' else .9)
                lead=clamp(along+lookahead,0,L)
                goalx,goaly=ax0+ux*lead,ay0+uy*lead
            else:goalx,goaly=target[0],target[1]
            rising=target[2]>=6.8 and nav_xyz[2]<target[2]-.35
            desired[0]=nav_xy[0] if rising else goalx
            desired[1]=nav_xy[1] if rising else goaly
            if altitude_slew_m_s is None:
                desired[2]=target[2]
            else:
                # A bounded command, not a claim that actual vertical speed
                # or stopping distance are proven safe. Control period 20 ms.
                desired[2]+=clamp(target[2]-desired[2],
                                   -.02*altitude_slew_m_s,
                                   .02*altitude_slew_m_s)
            reference_state=None
            if trajectory_reference is not None:
                # Reference time starts only when the UAV has first reached
                # the reference launch pose. This avoids pretending that the
                # MuJoCo craft was initialized in hover at z=4m.
                init=trajectory_reference.samples[0]
                if reference_start_t is None and (
                    math.dist(nav_xyz,init.xyz)<.50 and speed3<.65):
                    reference_start_t=t
                if reference_start_t is None:
                    reference_state=init
                else:
                    t_ref=max(0.,t-reference_start_t)
                    samples=trajectory_reference.samples
                    while (reference_index+1<len(samples)-1 and
                           samples[reference_index+1].t_s<=t_ref):
                        reference_index+=1
                    prev=samples[reference_index]
                    nxt=samples[min(reference_index+1,len(samples)-1)]
                    f_ref=clamp((t_ref-prev.t_s)/max(1e-8,nxt.t_s-prev.t_s),0.,1.)
                    from mars_lora.kinodynamic_reference import ReferenceSample
                    reference_state=ReferenceSample(
                        t_ref,
                        tuple(prev.xyz[j]*(1-f_ref)+nxt.xyz[j]*f_ref for j in range(3)),
                        tuple(prev.velocity_xyz[j]*(1-f_ref)+nxt.velocity_xyz[j]*f_ref for j in range(3)),
                        tuple(prev.acceleration_xyz[j]*(1-f_ref)+nxt.acceleration_xyz[j]*f_ref for j in range(3)),
                        (0.,0.,0.))
                    if t_ref>=trajectory_reference.duration_s:
                        idx=len(waypoints)-1
                        if reference_settle_s is None:reference_settle_s=t
                desired[:]=list(reference_state.xyz)
            if controller_mode=='high_authority':
                gain,damping,accel_limit,tilt_limit=1.8,1.85,2.7,.28
            elif controller_mode in ('barrier_guard','barrier_ttc'):
                # Candidate: same nominal control authority as high_authority,
                # with acceleration projected away from threatening structure.
                gain,damping,accel_limit,tilt_limit=1.8,1.85,2.7,.28
            elif controller_mode=='integral_guard':
                # Experimental bounded integral disturbance rejection:
                # persistent offsets can counter smooth constant crosswind,
                # but integral state is reset at waypoint handoffs and
                # suspended near walls to avoid windup into obstacles.
                gain,damping,accel_limit,tilt_limit=1.45,1.95,2.25,.23
                clearance_now=fast_city_clearance(p[:3],.08 if airframe=='crazyflie' else .32,grid=scene_grid)
                safe_to_integrate=clearance_now>1.5 and math.hypot(v[0],v[1])<2.5
                if safe_to_integrate:
                    x_integral=clamp(x_integral+.02*(desired[0]-p[0]),-.75,.75)
                    y_integral=clamp(y_integral+.02*(desired[1]-p[1]),-.75,.75)
                else:
                    x_integral*=.95;y_integral*=.95
                # Near structure, scale permitted acceleration down;
                # collisions are still measured independently of this.
                if clearance_now<1.0:
                    safety_holds+=1
                    accel_limit=min(accel_limit,1.1)
            else:
                gain,damping,accel_limit,tilt_limit=1.0,1.65,1.2,.16
            xi=x_integral if controller_mode=='integral_guard' else 0.
            yi=y_integral if controller_mode=='integral_guard' else 0.
            if reference_state is not None:
                ax=clamp(reference_state.acceleration_xyz[0]+gain*(desired[0]-nav_xy[0])+damping*(reference_state.velocity_xyz[0]-nav_velocity[0]),-accel_limit,accel_limit)
                ay=clamp(reference_state.acceleration_xyz[1]+gain*(desired[1]-nav_xy[1])+damping*(reference_state.velocity_xyz[1]-nav_velocity[1]),-accel_limit,accel_limit)
            elif trajectory_speed_limits is None:
                ax=clamp(gain*(desired[0]-nav_xy[0])-damping*nav_velocity[0]+.9*xi,-accel_limit,accel_limit)
                ay=clamp(gain*(desired[1]-nav_xy[1])-damping*nav_velocity[1]+.9*yi,-accel_limit,accel_limit)
            else:
                # Opt-in velocity-mode corner approach; use estimated state
                # only. These are command constraints, NOT realized speed /
                # jerk bounds or a robust reachable-set certificate.
                ex=desired[0]-nav_xy[0];ey=desired[1]-nav_xy[1]
                dist_xy=math.hypot(ex,ey)
                corner_cap=max(.30,trajectory_speed_limits[idx])
                target_cap=min(corner_cap,math.sqrt(max(0.,2*.65*dist_xy)))
                if rising:target_cap=0.
                if dist_xy>1.e-7:
                    target_vx=target_cap*ex/dist_xy
                    target_vy=target_cap*ey/dist_xy
                else:
                    target_vx=target_vy=0.
                ax=clamp(1.9*(target_vx-nav_velocity[0]),-accel_limit,accel_limit)
                ay=clamp(1.9*(target_vy-nav_velocity[1]),-accel_limit,accel_limit)
            if controller_mode in ('barrier_guard','barrier_ttc'):
                # Experimental distance-barrier projection, not a proof of
                # forward-invariant safety under thrust and wind saturation.
                radius=.08 if airframe=='crazyflie' else .32
                clearance_now=fast_city_clearance(p[:3],radius,grid=scene_grid)
                # finite differences of exact AABB signed distance, from
                # the same physical environment used by MuJoCo collisions.
                if clearance_now<2.2:
                    step=.18
                    px=(p[0]+step,p[1],p[2]);mxp=(p[0]-step,p[1],p[2])
                    py=(p[0],p[1]+step,p[2]);myp=(p[0],p[1]-step,p[2])
                    nx=(fast_city_clearance(px,radius,grid=scene_grid)-fast_city_clearance(mxp,radius,grid=scene_grid))/(2*step)
                    ny=(fast_city_clearance(py,radius,grid=scene_grid)-fast_city_clearance(myp,radius,grid=scene_grid))/(2*step)
                    norm=math.hypot(nx,ny)
                    if norm>1e-5:
                        nx/=norm;ny/=norm
                        v_normal=nx*v[0]+ny*v[1]
                        nominal_normal=nx*ax+ny*ay
                        # Two-term control barrier: prevent a negative
                        # closing velocity with a large inward acceleration.
                        if controller_mode=='barrier_ttc':
                            # CBF-inspired soft braking based on approach
                            # velocity; allow travel through legal corridors.
                            # This remains empirical and non-certified.
                            required=clamp(1.5*(.65-clearance_now)-1.6*v_normal,
                                           -2.7,2.7)
                        else:
                            required=clamp(2.3*(1.35-clearance_now)-2.1*v_normal,
                                           -2.7,2.7)
                        if nominal_normal<required:
                            correction=required-nominal_normal
                            ax+=correction*nx;ay+=correction*ny
                            size=math.hypot(ax,ay)
                            if size>accel_limit:
                                ax*=accel_limit/size;ay*=accel_limit/size
                            safety_holds+=1
            bx=math.cos(nav_yaw)*ax+math.sin(nav_yaw)*ay
            by=-math.sin(nav_yaw)*ax+math.cos(nav_yaw)*ay
            rt=clamp(-by/9.81,-tilt_limit,tilt_limit)
            pt=clamp(bx/9.81,-tilt_limit,tilt_limit)
            tilt_factor=max(.8,math.cos(roll)*math.cos(pitch))
            if reference_state is not None:
                zacc=(9.81+reference_state.acceleration_xyz[2]+
                      3*(desired[2]-nav_xyz[2])+
                      2.9*(reference_state.velocity_xyz[2]-nav_velocity[2]))
            else:
                zacc=9.81+3*(desired[2]-nav_xyz[2])-2.9*nav_velocity[2]
            force=mass*clamp(zacc/tilt_factor,1,18)
            mx=moment_scale*clamp(.95*(rt-roll)-.32*v[3],-.55,.55)
            my=moment_scale*clamp(.95*(pt-pitch)-.32*v[4],-.55,.55)
            arms=((armx,army),(armx,-army),(-armx,-army),(-armx,army)) if airframe=="crazyflie" else ((-armx,-army),(-armx,army),(armx,army),(armx,-army))
            for i,(x,y) in enumerate(arms):
                yaw_moment=0 if airframe=="crazyflie" else clamp(-.01*yaw-.008*v[5],-.02,.02)
                yaw_trim=0 if airframe=="crazyflie" else yaw_moment*(-1 if i%2==0 else 1)/(4*.0201)
                demand=clamp(force/4+mx*y/(4*army*army)-my*x/(4*armx*armx)+yaw_trim,0,max_thrust)
                rotor[i]+=(1-math.exp(-.02/(.045 if airframe=='crazyflie' else .08)))*(demand-rotor[i])
                data.ctrl[i]=clamp(rotor[i],0,max_thrust)
            data.xfrc_applied[1,0]=wind*(.07 if airframe=='crazyflie' else 1)*math.sin(t*.6+wind_phase)-.18*mass*v[0]
            data.xfrc_applied[1,1]=wind*(.07 if airframe=='crazyflie' else 1)*math.cos(t*.43+.73*wind_phase)-.18*mass*v[1]
            data.xfrc_applied[1,2]=-.12*mass*v[2]
            tilt=math.degrees(math.hypot(roll,pitch))
            max_tilt=max(max_tilt,tilt)
        mujoco.mj_step(model,data)
        # Check every physics step, not just the downsampled CSV rate.
        # The prior audit only checked the industrial fixtures, ignoring
        # city buildings present in the actual MuJoCo collision model.
        clearance=fast_city_clearance(data.qpos[:3],.08 if airframe=='crazyflie' else .32,grid=scene_grid)
        if clearance<minimum:
            minimum_clearance_event={"t_s":round(float(data.time),3),
              "position_xyz":[round(float(a),3) for a in data.qpos[:3]],
              "clearance_m":round(float(clearance),3),
              "estimated_position_xyz":([round(float(a),3) for a in nav_xyz]
                                         if 'nav_xyz' in locals() else None),
              "waypoint_index":idx}
        minimum=min(minimum,clearance)
        if clearance<.25 and first_clearance_violation is None:
            first_clearance_violation={"t_s":round(float(data.time),3),
               "position_xyz":[round(float(a),3) for a in data.qpos[:3]],
               "clearance_m":round(float(clearance),3),"waypoint_index":idx}
        if data.ncon>0 and first_contact is None:
            first_contact={"t_s":round(float(data.time),3),
               "position_xyz":[round(float(a),3) for a in data.qpos[:3]],
               "estimated_xyz":[round(float(a),3) for a in nav_xyz],
               "estimated_velocity_xyz":[round(float(a),3) for a in nav_velocity],
               "actual_velocity_xyz":[round(float(a),3) for a in data.qvel[:3]],
               "localization_error_m":round(float(math.dist(nav_xyz,data.qpos[:3])),3),
               "body15_attitude_error_deg":(
                   round(math.degrees(float(math.sqrt(sum(a*a for a in quat_body_delta(
                       tuple(body15.quat_wb),tuple(data.qpos[3:7])))))),3)
                   if body15 is not None else None),
               "body15_accel_bias_estimate":(
                   [round(float(a),5) for a in body15.accel_bias] if body15 is not None else None),
               "body15_gyro_bias_estimate":(
                   [round(float(a),6) for a in body15.gyro_bias] if body15 is not None else None),
               "body15_position_error_m":(round(float(math.dist(body15.position,data.qpos[:3])),3)
                                            if body15 is not None else None),
               "body15_position_sigma_rms_m":(round(float(body15.sigma_position_rms_m),3)
                                               if body15 is not None else None),
               "shadow_eskf_xyz":([round(float(x),3) for x in shadow.position]
                                   if shadow is not None else None),
               "shadow_eskf_error_m":(round(float(math.dist(shadow.position,data.qpos[:3])),3)
                                       if shadow is not None else None),
               "shadow_eskf_sigma_rms_m":(round(float(shadow.position_rms_sigma_m),3)
                                           if shadow is not None else None),
               "precontact_navigation_rmse_m":round(math.sqrt(localization_sq_sum/max(1,localization_updates)),3),
               "estimated_sigma_m":(round(float(inertial.sigma_m),3) if inertial else None),
               "reference_elapsed_s":(round(float(data.time-reference_start_t),3)
                    if reference_start_t is not None else None),
               "clearance_m":round(float(clearance),3),"waypoint_index":idx}
        contacts+=int(data.ncon>0)
        if k%25==0:
            p=data.qpos;v=data.qvel
            err=math.sqrt(sum((desired[i]-p[i])**2 for i in range(3)))
            max_error=max(max_error,err)
            cross_track.append(math.hypot(p[0]-desired[0],p[1]-desired[1]))
            records.append((round(float(data.time),4),*map(float,p[:3]),idx,clearance,data.ncon,tilt,err))
        if not all(math.isfinite(x) for x in data.qpos):
            raise RuntimeError("Non-finite MuJoCo pose")
        if idx==len(waypoints)-1 and math.dist(p[:3],waypoints[-1])<.5 and math.sqrt(sum(x*x for x in v[:3]))<.65:
            break
    if out:
        file=Path(out);file.parent.mkdir(parents=True,exist_ok=True)
        with file.open("w",newline="") as f:
            writer=csv.writer(f);writer.writerow(("t_s","x_m","y_m","z_m","waypoint","obstacle_clearance_m","contact_count","tilt_deg","tracking_error_m"))
            writer.writerows(records)
    result=dict(airframe=airframe,controller_mode=controller_mode,city_seed=city_seed,wind_phase=round(float(wind_phase),3),
                navigation_ate_rmse_m=round(math.sqrt(localization_sq_sum/max(1,localization_updates)),4),
                navigation_max_error_m=round(localization_max_error,4),
                eskf_shadow_enabled=eskf_shadow,
                eskf15_shadow_enabled=eskf15_shadow,
                pose6_shadow_enabled=pose6_shadow,
                pose6_observability_timeline=pose6_rows,
                pose6_full_rank_scans=pose6_full_rank,
                pose6_partial_scans=pose6_partial,
                pose6_rank_histogram=pose6_rank_hist,
                lidar_observable_scans=observable_scan_count,
                lidar_unobservable_scans=unobservable_scan_count,
                lidar_observability_rank_histogram=observable_ranks,
                lidar_longest_observation_gap_s=round(longest_observation_gap_s,3),
                lidar_first_unobservable_scan_t=first_unobservable_scan,
                lidar_first_high_residual_scan_t=first_high_residual_scan,
                lidar_observability_timeline=scan_observability_rows,
                eskf15_error_1m_first=body15_first_error_1m,
                eskf15_error_5m_first=body15_first_error_5m,
                eskf15_precontact_attitude_rmse_deg=(
                    round(math.degrees(math.sqrt(
                        body15_precontact_attitude_squared/max(1,body15_precontact_attitude_count))),3)
                    if body15 is not None else None),
                eskf15_shadow_rmse_m=(round(math.sqrt(body15_errsq/max(1,body15_samples)),4)
                                      if body15 is not None else None),
                eskf15_shadow_precontact_rmse_m=(
                    round(math.sqrt(body15_precontact_sq/max(1,body15_precontact_samples)),4)
                    if body15 is not None else None),
                eskf15_shadow_max_error_m=(round(body15_max_error,4) if body15 is not None else None),
                eskf15_shadow_cov99=(round(body15_coverage99/max(1,body15_samples),4)
                                     if body15 is not None else None),
                eskf15_shadow_sigma_position_rms_m=(
                    round(body15.sigma_position_rms_m,4) if body15 is not None else None),
                eskf15_lidar3d_accepted=body15_lidar3d_accept,
                eskf15_lidar3d_rejected=body15_lidar3d_reject,
                eskf15_planar_accepted=body15_planar_accept,
                eskf15_planar_rejected=body15_planar_reject,
                legacy_precontact_rmse_m=round(math.sqrt(legacy_precontact_squared/max(1,legacy_precontact_count)),4),
                eskf_shadow_precontact_rmse_m=(round(math.sqrt(shadow_precontact_squared/max(1,shadow_precontact_count)),4)
                                               if shadow is not None else None),
                eskf_shadow_ate_rmse_m=(round(math.sqrt(shadow_error_squared/max(1,shadow_samples)),4)
                                        if shadow is not None else None),
                eskf_shadow_max_error_m=(round(shadow_error_max,4) if shadow is not None else None),
                eskf_shadow_99pct_coverage=(round(shadow_nees_99_inside/max(1,shadow_samples),4)
                                              if shadow is not None else None),
                eskf_shadow_position_rms_sigma_m=(round(shadow.position_rms_sigma_m,4)
                                                    if shadow is not None else None),
                eskf_shadow_scan_accepted=shadow_scan_accepted,
                eskf_shadow_scan_rejected=shadow_scan_rejected,
                eskf_shadow_planar_accepted=shadow_planar_accepted,
                eskf_shadow_planar_rejected=shadow_planar_rejected,
                localization_drift_m_sqrt_s=localization_drift_m_sqrt_s,
                localization_measurement_sigma_m=localization_noise_sigma_m,
                localization_seed=localization_seed,
                localization_mode=localization_mode,
                dynamic_replanning=dynamic_replanning,
                lidar_blackout_window=lidar_blackout_window,
                map_replans=map_replans,map_replan_failures=map_replan_failures,
                replan_stopping_envelope=replan_stopping_envelope,
                stopping_envelope_rejections=stopping_envelope_rejections,
                altitude_slew_m_s=altitude_slew_m_s,
                trajectory_speed_profile_enabled=trajectory_speed_limits is not None,
                trajectory_reference_enabled=trajectory_reference is not None,
                online_risk_monitor=online_risk_monitor,
                risk_monitor_checks=risk_monitor_checks,
                risk_alarm_count=risk_alarm_count,
                loss_of_confidence_first=loss_of_confidence_first,
                loss_of_confidence_samples=loss_of_confidence_samples,
                first_risk_alarm=risk_alarm_first,
                reference_started=reference_start_t is not None,
                reference_settle_s=reference_settle_s,
                estimated_sigma_m=(round(inertial.sigma_m,4) if inertial is not None else None),
                lidar_matches_accepted=lidar_accepted,
                lidar_matches_rejected=lidar_rejected,
                safety_holds=safety_holds,completed=idx==len(waypoints)-1 and math.dist(data.qpos[:3],waypoints[-1])<.5,
                elapsed_s=round(float(data.time),2),waypoint_reached=idx+1,
                min_clearance_m=round(minimum,3),contact_samples=contacts,max_tilt_deg=round(max_tilt,2),
                max_tracking_error_m=round(max_error,2),
                xy_tracking_rms_m=round(math.sqrt(sum(x*x for x in cross_track)/max(1,len(cross_track))),3),
                first_contact=first_contact,
                first_clearance_violation=first_clearance_violation,
                minimum_clearance_event=minimum_clearance_event,
                accepted_replan_events=accepted_replan_events,
                rejected_replan_events=rejected_replan_events,
                route_waypoints=len(waypoints),end_position_xyz=[round(float(a),3) for a in data.qpos[:3]])
    if require_completion and (not result['completed'] or contacts>0 or minimum<.25):
        raise RuntimeError('Route failed native MuJoCo safety validation: '+str(result))
    return result
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="results/flight-validation")
    args=p.parse_args()
    results=[]
    for name in ("heavy","crazyflie"):
        outcome=run(name,out=f"{args.output}/{name}.csv")
        print(json.dumps(outcome))
        results.append(outcome)
        if not outcome["completed"] or outcome["min_clearance_m"]<.1 or outcome["contact_samples"]>0:
            raise SystemExit("Full mission failed validation: "+str(outcome))
    Path(args.output,"summary.json").write_text(json.dumps(results,indent=2)+"\n")
if __name__=="__main__":
    main()
