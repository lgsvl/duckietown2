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

import sys
import argparse

import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import Twist2DStamped, BoolStamped


class CarCmdSwitchNode(Node):
    def __init__(self, args):
        self.node_name = 'car_cmd_switch_node'
        super().__init__(self.node_name)

        self.args = args

        self.joystick_override = True

        self.pub_cmd = self.create_publisher(Twist2DStamped, self.args.publish_topic)

        self.sub_joy_cmd = self.create_subscription(Twist2DStamped, self.args.subscribe_topic1, self.joy_cmd_callback)
        self.sub_car_cmd = self.create_subscription(Twist2DStamped, self.args.subscribe_topic2, self.car_cmd_callback)
        self.sub_joystick_switch = self.create_subscription(BoolStamped, self.args.subscribe_topic_switch, self.joystick_override_callback)
        
    def joy_cmd_callback(self, joy_cmd_msg):
        if self.joystick_override:
            self.pub_cmd.publish(joy_cmd_msg)            

    def car_cmd_callback(self, car_cmd_msg):
        if not self.joystick_override:
            self.pub_cmd.publish(car_cmd_msg)    

    def joystick_override_callback(self, joystick_override_msg):
        self.loginfo("Switching to joystick mode: " + str(joystick_override_msg.data))
        self.joystick_override = joystick_override_msg.data

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        super().destroy_node()

    def loginfo(self, s):
        self.get_logger().info('%s' %(s))

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--subscribe_topic1",
                        type=str,
                        default="/joy_mapper_node/car_cmd",
                        help="topic to subscribe to for first car command option")
    parser.add_argument("--subscribe_topic2",
                        type=str,
                        default="/lane_controller_node/car_cmd",
                        help="topic to subscribe to for second car command option")
    parser.add_argument("--subscribe_topic_switch",
                        type=str,
                        default="/joystick_override",
                        help="topic to subscribe to for switching between car command options")
    parser.add_argument("--publish_topic",
                        type=str,
                        default="/car_cmd",
                        help="name of topic to publish actual car command to")

    args = parser.parse_args()
    node = CarCmdSwitchNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
