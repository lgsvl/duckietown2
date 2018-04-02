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
import time
import math

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped    


class JoyMapper(Node):

    def __init__(self):
        super().__init__('joy_mapper')

        self.joy = None
        self.last_pub_msg = None
        current_time = time.time()

        self.last_pub_time = Time()
        self.last_pub_time.sec = int(current_time)
        self.last_pub_time.nanosec = int(current_time%1 * 1E9)
        
        self.v_gain = 0.41
        self.omega_gain = 8.3
        self.bicycle_kinematics = False
        self.steer_angle_gain = 1
        self.simulated_vehicle_length = 0.18        
    
        self.sub = self.create_subscription(Joy, 'joy', self.cbJoy)

        self.pub_car_cmd = self.create_publisher(Twist2DStamped, "car_cmd")
        self.pub_joy_override = self.create_publisher(BoolStamped, "joystick_override")
        self.pub_parallel_autonomy = self.create_publisher(BoolStamped, "parallel_autonomy")
        self.pub_anti_instagram = self.create_publisher(BoolStamped, "anti_instagram_node/click")
        self.pub_e_stop = self.create_publisher(BoolStamped, "wheels_driver_node/emergency_stop")
        self.pub_avoidance = self.create_publisher(BoolStamped, "start_avoidance")
        
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)   

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.get_logger().info('I heard: [%s]' % str(joy_msg.axes))
        self.get_logger().info('I heard: [%s]' % str(joy_msg.buttons))
        self.publishControl()
        self.processButtons(joy_msg)
            
    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain
        if self.bicycle_kinematics:
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)

    def processButtons(self, joy_msg):

        if (joy_msg.buttons[6] == 1): #The back button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            self.get_logger().info('override_msg = True')
            self.pub_joy_override.publish(override_msg)
            
        elif (joy_msg.buttons[7] == 1): #the start button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            self.get_logger().info('override_msg = False')
            self.pub_joy_override.publish(override_msg)
            
        elif (joy_msg.buttons[5] == 1): # Right back button
            self.state_verbose ^= True
            self.get_logger().info('state_verbose = %s' % self.state_verbose)
            rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param
        elif (joy_msg.buttons[4] == 1): #Left back button
            self.state_parallel_autonomy ^= True
            self.get_logger().info('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            parallel_autonomy_msg = BoolStamped()
            parallel_autonomy_msg.header.stamp = self.joy.header.stamp
            parallel_autonomy_msg.data = self.state_parallel_autonomy
            self.pub_parallel_autonomy.publish(parallel_autonomy_msg)
        elif (joy_msg.buttons[3] == 1):
            anti_instagram_msg = BoolStamped()
            anti_instagram_msg.header.stamp = self.joy.header.stamp
            anti_instagram_msg.data = True
            self.get_logger().info('anti_instagram message')
            self.pub_anti_instagram.publish(anti_instagram_msg)
        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            self.get_logger().info('E-stop message')
            self.pub_e_stop.publish(e_stop_msg)
        elif (joy_msg.buttons[9] == 1): #push left joystick button
            avoidance_msg = BoolStamped()
            self.get_logger().info('start lane following with avoidance mode')
            avoidance_msg.header.stamp = self.joy.header.stamp
            avoidance_msg.data = True
            self.pub_avoidance.publish(avoidance_msg)
        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.get_logger().info('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = JoyMapper()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
