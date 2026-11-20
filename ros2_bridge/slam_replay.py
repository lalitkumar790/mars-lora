#!/usr/bin/env python3
"""Replay real MuJoCo rangefinder beams and *drifting* odometry to ROS 2.

Input: JSONL downloaded from browser's "Export LiDAR / odometry" button.
Outputs: /scan, /odom, /tf (odom->base_link and base_link->laser), /clock.
Source a supported ROS 2 distribution first (e.g., Jazzy).
The browser uses native MuJoCo rangefinder rays, but odometry is a noisy
pseudo-sensor: this does not validate a real GPS-denied navigation stack.

This file requires rclpy, sensor_msgs, nav_msgs, rosgraph_msgs, tf2_ros.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

def stamp(seconds):
    whole=math.floor(seconds)
    return Time(sec=int(whole),nanosec=int(round((seconds-whole)*1e9)))

def yaw_quaternion(yaw):
    return math.sin(yaw/2),math.cos(yaw/2)

class GPSDeniedReplay(Node):
    def __init__(self, records, speed):
        super().__init__('mars_lora_slam_replay')
        self.records=records
        self.speed=speed
        self.i=0
        self.sim_start=float(records[0]['t'])
        self.wall_start=time.monotonic()
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        fixed=TransformStamped()
        fixed.header.stamp=stamp(self.sim_start)
        fixed.header.frame_id='base_link'
        fixed.child_frame_id='laser'
        fixed.transform.translation.z=.01
        fixed.transform.rotation.w=1.
        self.static_tf.sendTransform(fixed)
        self.timer=self.create_timer(.02,self.tick)
        self.get_logger().info(f'Replaying {len(records)} range scans at {speed:g}x simulated speed')

    def tick(self):
        elapsed=(time.monotonic()-self.wall_start)*self.speed
        upto=self.sim_start+elapsed
        self.clock_pub.publish(Clock(clock=stamp(upto)))
        while self.i<len(self.records) and float(self.records[self.i]['t'])<=upto:
            self.publish_record(self.records[self.i])
            self.i+=1
        if self.i>=len(self.records):
            self.get_logger().info('Finished LiDAR/odometry replay')
            self.timer.cancel()
            self.done=True

    def publish_record(self,record):
        t=float(record['t'])
        header_stamp=stamp(t)
        ranges=[float(x) for x in record['ranges']]
        max_range=float(record['range_max'])
        scan=LaserScan()
        scan.header.stamp=header_stamp
        scan.header.frame_id='laser'
        scan.angle_min=float(record['angle_min'])
        scan.angle_increment=float(record['angle_increment'])
        scan.angle_max=scan.angle_min+scan.angle_increment*(len(ranges)-1)
        scan.time_increment=0.
        scan.scan_time=.5
        scan.range_min=float(record['range_min'])
        scan.range_max=max_range
        scan.ranges=[float('inf') if x>=max_range-.01 else x for x in ranges]
        self.scan_pub.publish(scan)
        odom_data=record['odom']
        x,y,yaw=(float(odom_data[k]) for k in ('x','y','yaw'))
        qz,qw=yaw_quaternion(yaw)
        odom=Odometry()
        odom.header.stamp=header_stamp
        odom.header.frame_id='odom'
        odom.child_frame_id='base_link'
        odom.pose.pose.position.x=x
        odom.pose.pose.position.y=y
        odom.pose.pose.orientation.z=qz
        odom.pose.pose.orientation.w=qw
        # Illustrative covariance; not sensor-calibrated.
        odom.pose.covariance[0]=.1
        odom.pose.covariance[7]=.1
        odom.pose.covariance[35]=.05
        self.odom_pub.publish(odom)
        transform=TransformStamped()
        transform.header=odom.header
        transform.child_frame_id='base_link'
        transform.transform.translation.x=x
        transform.transform.translation.y=y
        transform.transform.rotation.z=qz
        transform.transform.rotation.w=qw
        self.tf.sendTransform(transform)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('jsonl',type=Path,help='Browser exported mars-lora-lidar-odom.jsonl')
    parser.add_argument('--speed',type=float,default=1.)
    args=parser.parse_args()
    if args.speed<=0:parser.error('--speed must be positive')
    records=[json.loads(line) for line in args.jsonl.read_text().splitlines() if line.strip()]
    if not records:parser.error('Input LiDAR JSONL is empty')
    rclpy.init()
    node=GPSDeniedReplay(records,args.speed)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
