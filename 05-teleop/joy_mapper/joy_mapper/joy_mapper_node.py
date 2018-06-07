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
import time
import math

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped    

import RPi.GPIO as GPIO


OUT = 18  # GPIO pin number for IR sensor out signal
TRIG = 23  # GPIO pin number for Ultrasonic sensor trig signal
ECHO = 24  # GPIO pin number for Ultrasonic sensor echo signal


class JoyMapper(Node):

    def __init__(self, args):
        super().__init__('joy_mapper')

        self.args = args

        if args.use_cliff_detection:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(OUT, GPIO.IN)
            time.sleep(1)
            self.loginfo('Cliff detection mode is running. Drive safely.')

        if args.use_obstacle_detection:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(TRIG,GPIO.OUT)
            GPIO.setup(ECHO,GPIO.IN)
            GPIO.output(TRIG, False)
            time.sleep(2)
            self.loginfo('Obstacle detection mode is running. Drive safely.')

        self.joy = None
        self.last_pub_msg = None
        self.cliff_flag = True
        self.obstacle_flag = True

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

        car_cmd_topic = "car_cmd"
        if self.args.publish_topic:
            self.loginfo("Received publish topic argument, publishing Twist2DStamped to " + self.args.publish_topic)
            car_cmd_topic = self.args.publish_topic

        self.pub_car_cmd = self.create_publisher(Twist2DStamped, car_cmd_topic)
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
        #self.get_logger().info('I heard: [%s]' % str(joy_msg.axes))
        #self.get_logger().info('I heard: [%s]' % str(joy_msg.buttons))
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain

        if self.args.use_cliff_detection:
            car_cmd_msg = self.cliff_detection_handler(car_cmd_msg)

        if self.args.use_obstacle_detection:
            car_cmd_msg = self.obstacle_detection_handler(car_cmd_msg)

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
            #rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param
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

    def loginfo(self, s):
        self.get_logger().info('%s' % (s))      

    def cliff_detection_handler(self, car_cmd_msg):
        is_cliff_detected = GPIO.input(OUT)
        if is_cliff_detected:
            if self.cliff_flag:
                self.get_logger().info('Cliff detected ahead!')
                self.cliff_flag = False
            if car_cmd_msg.v > 0:
                car_cmd_msg.v = 0.
        else:
            if not self.cliff_flag:
                self.get_logger().info('Safe now. No cliff ahead.')
                self.cliff_flag = True

        return car_cmd_msg

    def obstacle_detection_handler(self, car_cmd_msg):
        if GPIO.input(ECHO) != 0:
            return car_cmd_msg

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_duration = None
        t0 = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start - t0 > 0.01:
                return car_cmd_msg

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            if pulse_duration > 0.01:
                return car_cmd_msg

        if pulse_duration is None:
            return car_cmd_msg

        distance = pulse_duration * 17150
        distance = round(distance, 2)

        if distance < 30:
            if self.obstacle_flag:
                self.get_logger().info('Obstacle detected in {} cm.'.format(distance))
                self.obstacle_flag = False
            if car_cmd_msg.v > 0:
                car_cmd_msg.v = 0.
        else:
            if not self.obstacle_flag:
                self.get_logger().info('Safe now. No obstacles ahead.')
                self.obstacle_flag = True

        return car_cmd_msg


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--publish_topic",
                        type=str,
                        help="topic name to publish car command on")
    parser.add_argument("--use_cliff_detection",
                        type=int,
                        help="1 if using ir sensor to detect cliff, 0 otherwise")
    parser.add_argument("--use_obstacle_detection",
                        type=int,
                        help="1 if using ultrasonic sensor to detect obstacle, 0 otherwise")
    args = parser.parse_args()

    node = JoyMapper(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
