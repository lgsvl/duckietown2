# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import sys
import time
import math

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from duckietown_msgs.msg import Twist2DStamped, LanePose 


class LaneControllerNode(Node):
    def __init__(self, args):
        self.node_name = 'lane_controller_node'
        super().__init__(self.node_name)
        
        self.args = args

        self.lane_reading = None

        self.setGains()
        
        publish_topic = "/car_cmd"
        if self.args.publish_topic:
            self.loginfo("Received publish topic argument, publishing Twist2DStamped to " + self.args.publish_topic)
            publish_topic = self.args.publish_topic
        self.pub_car_cmd = self.create_publisher(Twist2DStamped, publish_topic)
        self.sub_lane_reading = self.create_subscription(LanePose, "lane_pose", self.cbPose)

        self.loginfo("Initialized ")

    def setupParameter(self):
        """ fill out when we have parameter functionality"""
        return

    def setGains(self):
        #v_bar = 0.5 # nominal speed, 0.5m/s
        #k_theta = -2.0
        #k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        #theta_thres = math.pi / 6
        #d_thres = math.fabs(k_theta / k_d) * theta_thres
        #d_offset = 0.0
        #v_bar = 0.3864
        v_bar = 0.2
        k_d = -10.30
        k_theta = -5.15
        theta_thres = 0.523
        d_thres = 0.2615
        d_offset = 0.0        

        if self.args.gain:
            self.loginfo("Received gain argument, setting gain to " + str(self.args.gain))

        self.v_bar = v_bar if not self.args.gain else self.args.gain  # Linear velocity
        self.k_d = k_d # P gain for theta
        self.k_theta = k_theta # P gain for d
        self.d_thres = d_thres # Cap for error in d
        self.theta_thres = theta_thres # Maximum desire theta
        self.d_offset = d_offset # a configurable offset from the lane position
        
    def getGains_event(self, event):
        """ fill out when we have parameter functionality"""
        return

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

    def cbPose(self, lane_pose_msg):
        self.lane_reading = lane_pose_msg

        #current_time = time.time()
        #timestamp_now = Time()
        #timestamp_now.sec = int(current_time)
        #timestamp_now.nanosec = int(current_time%1 * 1E9)
        #image_delay_stamp = Time()
        #image_delay_stamp.sec = timestamp_now.sec - self.lane_reading.header.stamp.sec
        #image_delay_stamp.nanosec = timestamp_now.nanosec - self.lane_reading.header.stamp.nanosec

        #image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive        

        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        car_control_msg.omega = self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative

        self.publishCmd(car_control_msg)
        
    def loginfo(self, s):
        self.get_logger().info('%s' % (s))


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--gain", 
                        type=float,
                        help="gain value for motor speed")
    parser.add_argument("--publish_topic", 
                        type=str,
                        help="topic name to publish car command to")
    args = parser.parse_args()
    
    node = LaneControllerNode(args)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
