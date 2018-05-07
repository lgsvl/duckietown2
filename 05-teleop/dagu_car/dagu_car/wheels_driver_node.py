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

import time
import sys

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_car_include.dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(Node):
    def __init__(self):
        self.node_name = 'wheels_driver_node'
        super().__init__(self.node_name)

        self.estop=False

        self.driver = DaguWheelsDriver()
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, 'wheels_cmd_executed')

        self.control_constant = 1.0

        self.sub_topic = self.create_subscription(WheelsCmdStamped, 'wheels_cmd', self.cbWheelsCmd)
        self.sub_e_stop = self.create_subscription(BoolStamped, 'wheels_driver_node/emergency_stop', self.cbEStop)

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.setWheelsSpeed(left=0.0,right=0.0)
            return
        
        self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver

        current_time = time.time()
        timestamp = Time()
        timestamp.sec = int(current_time)
        timestamp.nanosec = int(current_time%1 * 1E9)
        self.msg_wheels_cmd.header.stamp = timestamp
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd) 

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            self.get_logger().info("[%s] Emergency Stop Activated")
            self.driver.setWheelsSpeed(left=0.0,right=0.0)
        else:
            self.get_logger().info("[%s] Emergency Stop Released")

    def destroy_node(self):
        self.get_logger().info("Shutting down motors")
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        super().destroy_node()


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = WheelsDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
