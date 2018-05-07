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
import sys
import os.path
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped    
from duckietown_msgs.srv import SetValue
from std_srvs.srv import Empty


class InverseKinematicsNode(Node):
    def __init__(self, args):
        self.node_name = 'inverse_kinematics_node'
        super().__init__(self.node_name)

        self.args = args

        self.v_gain = 0.41
        self.omega_gain = 8.3
        self.bicycle_kinematics = False
        self.steer_angle_gain = 1
        self.simulated_vehicle_length = 0.18        
   
        self.gain = 0.65
        self.trim = 0.0
        self.baseline = 0.1
        self.radius = 0.0318
        self.k = 27.0
        self.limit = 1.0
        self.limit_max = 1.0
        self.limit_min = 0.0

        self.srv_set_gain = self.create_service(SetValue, 'set_gain', self.cbSrvSetGain)   
        self.srv_set_trim = self.create_service(SetValue, 'set_trim', self.cbSrvSetTrim)   
        self.srv_set_baseline = self.create_service(SetValue, 'set_baseline', self.cbSrvSetBaseline)   
        self.srv_set_radius = self.create_service(SetValue, 'set_radius', self.cbSrvSetRadius)   
        self.srv_set_k = self.create_service(SetValue, 'set_trim', self.cbSrvSetTrim)   
        self.srv_set_limit = self.create_service(SetValue, 'set_trim', self.cbSrvSetTrim)   
        #self.srv_save = self.create_service(Empty, 'save_calibration', self.cbSrvSaveCalibration)   

        self.sub_car_cmd = self.create_subscription(Twist2DStamped, "/car_cmd", self.car_cmd_callback)
        
        publish_topic = "wheels_cmd"

        if self.args.publish_topic:
            self.loginfo("Received publish topic argument, publishing WheelsCmdStamped to " + self.args.publish_topic)
            publish_topic = self.args.publish_topic

        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, publish_topic)

        self.get_logger().info('[%s] Initialized.' % self.node_name)
        self.printValues()

    """
    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
            "limit": self.limit,
        }
        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        self.get_logger().info("[%s] Saved to %s" %(self.node_name, file_name))
    """

    def cbSrvSaveCalibration(self, req):
        return EmptyResponse()

    def cbSrvSetGain(self, req):
        self.gain = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetTrim(self, req):
        self.trim = req.value
        self.printValues()
        return SetValueResponse()
    
    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        self.printValues()
        return SetValueResponse()
    
    def cbSrvSetRadius(self, req):
        self.radius = req.value
        self.printValues()
        return SetValueResponse()
    
    def cbSrvSetK(self, req):
        self.k = req.value
        self.printValues()
        return SetValueResponse()
    
    def cbSrvSetLimit(self, req):
        self.limit = self.setLimit(req.value)
        self.printValues()
        return SetValueResponse()

    def setLimit(self, value):
        if value > self.limit_max:
            self.get_logger().warn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            self.get_logger().warn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit

    def printValues(self):
        self.get_logger().info("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def car_cmd_callback(self, msg_car_cmd):
        k_r = self.k
        k_l = self.k

        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        
        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv
        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)
        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)
       
        # measure delay 
        #current_time = time.time()
        #message_time = msg_wheels_cmd.header.stamp.sec + msg_wheels_cmd.header.stamp.nanosec*1e-9
        #delay = current_time - message_time
        #print("wheels_cmd timestamp: " + str(msg_wheels_cmd.header.stamp))
        #print("current time: " + str(current_time))
        #print("delay: " + str(delay))

    def loginfo(self, s):
        self.get_logger().info('%s' % (s))           

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--publish_topic",
                        type=str,
                        help="topic name to publish wheels command on")
    args = parser.parse_args()

    node = InverseKinematicsNode(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
