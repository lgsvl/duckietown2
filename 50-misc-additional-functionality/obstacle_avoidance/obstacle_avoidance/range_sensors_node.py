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
from sensor_msgs.msg import Joy, Range
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from duckietown.duckietown_utils.time import get_current_time_msg

import RPi.GPIO as GPIO

OUT = 18  # GPIO pin number for IR sensor out signal
TRIG = 23  # GPIO pin number for Ultrasonic sensor trig signal
ECHO = 24  # GPIO pin number for Ultrasonic sensor echo signal

ULTRASOUND = 0    # type of radiation sensor, ultrasonic distance
INFRARED = 1      # type of radiation sensor, IR binary distance
ULTRASOUND_DETECTION_THRESHOLD = 30 # distance threshold (cm) to register obstacle


class RangeSensors(Node):

    def __init__(self, args):
        super().__init__('range_sensors_node')

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

        self.cliff_flag = True
        self.obstacle_flag = True

        self.pub_e_stop = self.create_publisher(BoolStamped, "wheels_driver_node/emergency_stop")
        self.pub_range = self.create_publisher(Range, "distance_to_obstacle")

    def publishRange(self):
        if self.args.use_cliff_detection:
            range_msg = self.cliff_detection_handler()
        if self.args.use_obstacle_detection:
            range_msg = self.obstacle_detection_handler()
        self.pub_range.publish(range_msg)

    def publishRange(self, timestamp, sensor_type, distance):
        range_msg = Range()
        range_msg.header.stamp = timestamp
        range_msg.radiation_type = sensor_type
        range_msg.range = 0.0
        self.pub_range.publish(range_msg)

    def publishStop(self):
        e_stop_msg = BoolStamped()
        e_stop_msg.header.stamp = get_current_time_msg()
        e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
        self.get_logger().info('E-stop message')
        self.pub_e_stop.publish(e_stop_msg)

    def cliff_detection_handler(self):
        timestamp = get_current_time_msg()

        is_cliff_detected = GPIO.input(OUT)
        if is_cliff_detected:
            if self.cliff_flag:     # first time cliff detected
                self.get_logger().info('Cliff detected ahead!')
                self.cliff_flag = False
                self.publishStop()
                self.publishRange(timestamp, ULTRASOUND, 1.0)
        else:
            if not self.cliff_flag:
                self.get_logger().info('Safe now. No cliff ahead.')
                self.cliff_flag = True
                self.publishStop()  # allow joystick drive again
                self.publishRange(timestamp, INFRARED, 0.0)

        return car_cmd_msg, range_msg

    def obstacle_detection_handler(self):
        timestamp = get_current_time_msg()

        if GPIO.input(ECHO) != 0:
            return

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_duration = None
        t0 = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start - t0 > 0.01:     # timeout
                return

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            if pulse_duration > 0.01:
                return                      # timeout

        if pulse_duration is None:
            return

        distance = pulse_duration * 17150
        distance = round(distance, 2)

        if distance < ULTRASOUND_DETECTION_THRESHOLD:
            if self.obstacle_flag:      # first time obstacle detected
                self.get_logger().info('Obstacle detected in {} cm.'.format(distance))
                distance_in_m = distance / 100    # distance in m
                self.obstacle_flag = False
                self.publishStop()
                self.publishRange(timestamp, ULTRASOUND, distance_in_m)
        else:
            if not self.obstacle_flag:
                self.get_logger().info('Safe now. No obstacles ahead.')
                self.obstacle_flag = True
                self.publishStop()
                self.publishRange(timestamp, ULTRASOUND, 0.0)

    def loginfo(self, s):
        self.get_logger().info('%s' % (s))


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--use_cliff_detection",
                        type=int,
                        help="1 if using ir sensor to detect cliff, 0 otherwise")
    parser.add_argument("--use_obstacle_detection",
                        type=int,
                        help="1 if using ultrasonic sensor to detect obstacle, 0 otherwise")
    args = parser.parse_args()

    node = RangeSensors(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
