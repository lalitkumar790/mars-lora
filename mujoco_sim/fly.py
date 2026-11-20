"""Experimental MuJoCo quadrotor PID + ROS 2 PoseStamped publisher.

Requirements: pip install mujoco numpy; optional ROS 2 rclpy and geometry_msgs.
Run: python mujoco_sim/fly.py --seconds 30 --output results/flight.csv
Run with --ros to publish /mars_lora/uav_pose (requires sourced ROS 2).
This example is NOT validated stable-flight or navigation performance.
"""
import argparse
import csv
import math
from pathlib import Path
import numpy as np
import mujoco

def clamp(x,lo,hi):
    return float(max(lo,min(hi,x)))

def quat_to_rpy(q):
    w,x,y,z=map(float,q)
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(clamp(2*(w*y-z*x),-1,1))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return roll,pitch,yaw

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--seconds",type=float,default=30)
    parser.add_argument("--output",default="results/flight.csv")
    parser.add_argument("--ros",action="store_true")
    args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("quad.xml")))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    node=pub=None
    if args.ros:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        rclpy.init()
        node=rclpy.create_node("mars_lora_mujoco")
        pub=node.create_publisher(PoseStamped,"/mars_lora/uav_pose",10)
    target=np.array([0.0,0.0,2.5])
    controls_period=0.02
    log_period=0.1
    next_control=next_log=0.0
    output=Path(args.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["t_s","x_m","y_m","z_m","qw","qx","qy","qz"])
        while data.time < args.seconds:
            t=float(data.time)
            if t >= next_control-1e-8:
                # A small horizontal step tests whether position and attitude control
                # recover. The building inspection route is not yet integrated.
                if t > args.seconds/3:
                    target[0]=1.5
                if t > 2*args.seconds/3:
                    target[1]=1.0
                pos=np.asarray(data.qpos[:3])
                vel=np.asarray(data.qvel[:3])
                roll,pitch,yaw=quat_to_rpy(data.qpos[3:7])
                ax=clamp(1.1*(target[0]-pos[0])-1.4*vel[0],-1.5,1.5)
                ay=clamp(1.1*(target[1]-pos[1])-1.4*vel[1],-1.5,1.5)
                roll_target=clamp(-ay/9.81,-0.2,0.2)
                pitch_target=clamp(ax/9.81,-0.2,0.2)
                thrust=1.2*clamp(9.81+3.0*(target[2]-pos[2])-2.2*vel[2],1,18)
                # Approximate rotational-rate PD. Assumes yaw remains near zero.
                moments=np.array([
                    clamp(1.1*(roll_target-roll)-0.22*data.qvel[3],-0.6,0.6),
                    clamp(1.1*(pitch_target-pitch)-0.22*data.qvel[4],-0.6,0.6)
                ])
                a=0.21
                rotor_xy=((a,a),(a,-a),(-a,-a),(-a,a))
                # Mx = sum(y_i F_i); My = sum(-x_i F_i)
                for i,(x,y) in enumerate(rotor_xy):
                    data.ctrl[i]=clamp(thrust/4+moments[0]*y/(4*a*a)-moments[1]*x/(4*a*a),0,15)
                next_control+=controls_period
            mujoco.mj_step(model,data)
            if data.time >= next_log-1e-8:
                p=data.qpos
                w.writerow([round(float(data.time),5),*map(float,p[:7])])
                if pub:
                    msg=PoseStamped()
                    msg.header.frame_id="world"
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.pose.position.x=float(p[0])
                    msg.pose.position.y=float(p[1])
                    msg.pose.position.z=float(p[2])
                    msg.pose.orientation.w=float(p[3])
                    msg.pose.orientation.x=float(p[4])
                    msg.pose.orientation.y=float(p[5])
                    msg.pose.orientation.z=float(p[6])
                    pub.publish(msg)
                next_log+=log_period
    if node:
        node.destroy_node()
        import rclpy
        rclpy.shutdown()
    print("Flight pose log:",output,"(validate trajectory before using in publication)")

if __name__=="__main__":
    main()
