"""Full mission validation for browser MJCF airframes using actual MuJoCo physics.

This mirrors the browser controller's principal control law. It is NOT a
bit-for-bit browser execution nor a proof of flight safety.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import mujoco
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mars_lora.urban_scene import add_city_mjcf
from mars_lora.rf_astar import signed_clearance as full_city_clearance

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
def clamp(x,a,b):
    return max(a,min(b,x))
def quat_to_rpy(q):
    w,x,y,z=q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(clamp(2*(w*y-z*x),-1,1)),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def run(airframe,duration=240,wind=.25,out=None,route=None,require_completion=False,
        controller_mode='legacy',wind_phase=0.):
    if controller_mode not in ('legacy','high_authority','integral_guard'):
        raise ValueError('Unsupported controller mode')
    waypoints=list(ROUTE if route is None else route)
    if len(waypoints)<2:raise ValueError('A flight requires at least 2 waypoints')
    root=Path(__file__).resolve().parents[1]
    xml=(root/"public"/("crazyflie.xml" if airframe=="crazyflie" else "quad.xml")).read_text()
    model=mujoco.MjModel.from_xml_string(add_city_mjcf(xml))
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
    dt=model.opt.timestep
    steps=round(duration/dt)
    for k in range(steps):
        t=float(data.time)
        p=data.qpos;v=data.qvel
        if k%5==0:
            roll,pitch,yaw=quat_to_rpy(p[3:7])
            target=waypoints[idx]
            delta=[target[i]-p[i] for i in range(3)]
            speed3=math.sqrt(sum(v[i]*v[i] for i in range(3)))
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
                along=clamp((p[0]-ax0)*ux+(p[1]-ay0)*uy,0,L)
                lookahead=1.5 if controller_mode=='high_authority' else (1.1 if controller_mode=='integral_guard' else .9)
                lead=clamp(along+lookahead,0,L)
                goalx,goaly=ax0+ux*lead,ay0+uy*lead
            else:goalx,goaly=target[0],target[1]
            rising=target[2]>=6.8 and p[2]<target[2]-.35
            desired[0]=p[0] if rising else goalx
            desired[1]=p[1] if rising else goaly
            desired[2]=target[2]
            if controller_mode=='high_authority':
                gain,damping,accel_limit,tilt_limit=1.8,1.85,2.7,.28
            elif controller_mode=='integral_guard':
                # Experimental bounded integral disturbance rejection:
                # persistent offsets can counter smooth constant crosswind,
                # but integral state is reset at waypoint handoffs and
                # suspended near walls to avoid windup into obstacles.
                gain,damping,accel_limit,tilt_limit=1.45,1.95,2.25,.23
                clearance_now=full_city_clearance(p[:3])-(.08 if airframe=="crazyflie" else .32)
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
            ax=clamp(gain*(desired[0]-p[0])-damping*v[0]+.9*xi,-accel_limit,accel_limit)
            ay=clamp(gain*(desired[1]-p[1])-damping*v[1]+.9*yi,-accel_limit,accel_limit)
            bx=math.cos(yaw)*ax+math.sin(yaw)*ay
            by=-math.sin(yaw)*ax+math.cos(yaw)*ay
            rt=clamp(-by/9.81,-tilt_limit,tilt_limit)
            pt=clamp(bx/9.81,-tilt_limit,tilt_limit)
            tilt_factor=max(.8,math.cos(roll)*math.cos(pitch))
            force=mass*clamp((9.81+3*(desired[2]-p[2])-2.9*v[2])/tilt_factor,1,18)
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
        clearance=full_city_clearance(data.qpos[:3])-(.08 if airframe=='crazyflie' else .32)
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
    result=dict(airframe=airframe,controller_mode=controller_mode,wind_phase=round(float(wind_phase),3),
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
