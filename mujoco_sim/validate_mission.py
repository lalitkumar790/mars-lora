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
        localization_seed=0,localization_mode='drift',dynamic_replanning=False):
    if controller_mode not in ('legacy','high_authority','integral_guard','barrier_guard','barrier_ttc'):
        raise ValueError('Unsupported controller mode')
    if localization_mode not in ('drift','lidar_match','lidar_imu3d'):
        raise ValueError('Unknown navigation localization mode')
    if localization_drift_m_sqrt_s<0 or localization_noise_sigma_m<0:
        raise ValueError('Localization perturbation standard deviations must be nonnegative')
    waypoints=list(ROUTE if route is None else route)
    if len(waypoints)<2:raise ValueError('A flight requires at least 2 waypoints')
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
    imu_last_velocity=[float(x) for x in data.qvel[:3]]
    map_replans=map_replan_failures=localization_lost_events=0
    last_replan_time=-100.
    if localization_mode=='lidar_imu3d':
        inertial=InertialMapEstimator.initial(tuple(float(x) for x in data.qpos[:3]))
        inertial.process_sigma_m_sqrt_s=max(.012,localization_drift_m_sqrt_s)
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
            if localization_mode=='lidar_imu3d':
                # Synthetic IMU packet is generated from *finite differenced*
                # native velocity plus independent seeded sensor perturbation.
                # The estimator/controller only consume the noisy packet.
                # No ground-truth XY or body velocity enters this controller.
                measured_accel=[
                    (float(v[i])-imu_last_velocity[i])/localization_dt
                    +localization_bias[i] if i<2 else
                    (float(v[i])-imu_last_velocity[i])/localization_dt
                    for i in range(3)]
                measured_accel=[a+localization_rng.gauss(0.,.035)
                                for a in measured_accel]
                for axis in range(3):imu_last_velocity[axis]=float(v[axis])
                inertial.predict(measured_accel,
                                float(v[5])+localization_rng.gauss(0.,.003),
                                localization_dt)
                if t>=next_lidar_time:
                    reading=simulate_scan(tuple(float(x) for x in p[:3]),
                                          yaw,scene_solids,lidar_rng,
                                          noise=.06,dropout=.03)
                    accepted,scan_score,hits=inertial.correct(reading,scene_solids)
                    if accepted:lidar_accepted+=1
                    else:lidar_rejected+=1
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
            nav_xy=(raw_xy if localization_mode=='lidar_imu3d' else
                    (raw_xy[0]+map_correction[0],raw_xy[1]+map_correction[1]))
            if localization_mode!='lidar_imu3d':
                nav_xyz=(nav_xy[0],nav_xy[1],float(p[2]))
                nav_velocity=tuple(float(x) for x in v[:3])
                nav_yaw=yaw
            navigation_error=(math.dist(nav_xyz,p[:3]) if localization_mode=='lidar_imu3d'
                              else math.hypot(nav_xy[0]-p[0],nav_xy[1]-p[1]))
            localization_sq_sum+=navigation_error**2
            localization_updates+=1
            localization_max_error=max(localization_max_error,navigation_error)
            target=waypoints[idx]
            if localization_mode=='lidar_imu3d' and dynamic_replanning and (
                inertial.sigma_m>.20 and t-last_replan_time>24. and
                map_replans<3 and idx<len(waypoints)-1):
                # Dynamic uncertainty-triggered map-known A*: the estimator
                # drives start XYZ; truth is NEVER used for replanning.
                from mars_lora.rf_astar import plan
                try:
                    updated=plan('shortest',solids=scene_solids,
                        start_xyz=tuple(nav_xyz),goal_xyz=tuple(waypoints[-1]),
                        radius=(.08 if airframe=='crazyflie' else .32),
                        localization_sigma_m=min(.45,inertial.sigma_m),
                        max_nodes=16000)
                    waypoints=[tuple(nav_xyz)]+updated.points[1:]
                    idx=1;map_replans+=1
                    target=waypoints[idx]
                except (RuntimeError,ValueError,AssertionError):
                    map_replan_failures+=1
                last_replan_time=t
            delta=[target[i]-nav_xyz[i] for i in range(3)]
            speed3=math.sqrt(sum(nav_velocity[i]**2 for i in range(3)))
            if math.sqrt(sum(x*x for x in delta))<.5 and speed3<.65 and idx<len(waypoints)-1:
                idx+=1
                if controller_mode=='integral_guard':
                    x_integral=y_integral=0.
                target=waypoints[idx]
            # Match browser lookahead tracking: project current XY onto the
            # segment, advance by 0.9m, and brake at each corner.
            ax0,ay0=(0,0) if idx==0 else waypoints[idx-1][:2]
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
            desired[2]=target[2]
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
            ax=clamp(gain*(desired[0]-nav_xy[0])-damping*nav_velocity[0]+.9*xi,-accel_limit,accel_limit)
            ay=clamp(gain*(desired[1]-nav_xy[1])-damping*nav_velocity[1]+.9*yi,-accel_limit,accel_limit)
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
            force=mass*clamp((9.81+3*(desired[2]-nav_xyz[2])-2.9*nav_velocity[2])/tilt_factor,1,18)
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
        minimum=min(minimum,clearance)
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
                localization_drift_m_sqrt_s=localization_drift_m_sqrt_s,
                localization_measurement_sigma_m=localization_noise_sigma_m,
                localization_seed=localization_seed,
                localization_mode=localization_mode,
                dynamic_replanning=dynamic_replanning,
                map_replans=map_replans,map_replan_failures=map_replan_failures,
                estimated_sigma_m=(round(inertial.sigma_m,4) if inertial is not None else None),
                lidar_matches_accepted=lidar_accepted,
                lidar_matches_rejected=lidar_rejected,
                safety_holds=safety_holds,completed=idx==len(waypoints)-1 and math.dist(data.qpos[:3],waypoints[-1])<.5,
                elapsed_s=round(float(data.time),2),waypoint_reached=idx+1,
                min_clearance_m=round(minimum,3),contact_samples=contacts,max_tilt_deg=round(max_tilt,2),
                max_tracking_error_m=round(max_error,2),
                xy_tracking_rms_m=round(math.sqrt(sum(x*x for x in cross_track)/max(1,len(cross_track))),3),
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
