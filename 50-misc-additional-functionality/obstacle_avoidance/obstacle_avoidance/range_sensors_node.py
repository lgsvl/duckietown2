# Copyright (c) 2018 LG Electronics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import argparse
import sys
import time
import math

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Joy, Range
from duckietown.duckietown_utils.time import get_current_time_msg

# GPIO for ultrasound
import RPi.GPIO as GPIO

# I2C circuitpython for tof laser
import board
import busio
import adafruit_vl6180x

OUT = 18  # GPIO pin number for IR sensor out signal
TRIG = 23  # GPIO pin number for Ultrasonic sensor trig signal
ECHO = 24  # GPIO pin number for Ultrasonic sensor echo signal

ULTRASOUND = 0  # type of radiation sensor, ultrasonic distance (radiation_type)
INFRARED = 1    # type of radiation sensor, IR binary distance
TOF_LASER = 2   # type of radiation sensor, TOF laser distance


class RangeSensors(Node):

    def __init__(self, args):
        super().__init__('range_sensors_node')

        self.args = args
        self.last_time = 0.0
        self.last_distance = 999999.99
        self.tof_sensor_online = False

        if args.use_cliff_detection:
            self.startTOFSensor()
            time.sleep(1)
            self.loginfo('Cliff detection mode is running. Drive safely.')

        if args.use_obstacle_detection:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(TRIG,GPIO.OUT)
            GPIO.setup(ECHO,GPIO.IN)
            GPIO.output(TRIG, False)
            time.sleep(2)
            self.loginfo('Obstacle detection mode is running. Drive safely.')

        self.pub_ultrasound = self.create_publisher(Range, "/sensor/ultrasound")
        self.pub_tof = self.create_publisher(Range, "/sensor/tof")

        self.t = self.create_timer(0.1, self.update)

    def update(self):
        if self.args.use_cliff_detection:
            self.cliff_detection_handler()
        if self.args.use_obstacle_detection:
            self.obstacle_detection_handler()

    def publishRange(self, pub, sensor_type, distance):
        range_msg = Range()
        range_msg.header.stamp = get_current_time_msg()
        range_msg.radiation_type = sensor_type
        range_msg.range = distance
        pub.publish(range_msg)

    def get_tof_distance(self):
        try:
            # self.loginfo("reading sensor")
            # self.loginfo("status:" + str(self.tof_sensor.range_status))
            distance = float(self.tof_sensor.range)
            return True, distance
        except OSError as e:
            self.loginfo('Something is wrong with the TOF sensor')
            print(e)
            return False, -float("Inf")

    def cliff_detection_handler(self):
        if self.tof_sensor_online:
            tof_valid, cliff_distance = self.get_tof_distance()
            if not tof_valid:       # try to reconnect to sensor
                self.startTOFSensor()
                time.sleep(1)
            # self.loginfo("publishing distance " + str(cliff_distance))
            self.publishRange(self.pub_tof, INFRARED, cliff_distance)
        else:
            self.loginfo("sensor not online, trying to restart")
            self.startTOFSensor()
            self.publishRange(self.pub_tof, INFRARED, -float("Inf"))

    def get_ultrasound_distance(self):
        if GPIO.input(ECHO) != 0:
            return -1

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_duration = None
        pulse_start = time.time()
        t0 = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start - t0 > 0.01:     # timeout
                return -1

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            if pulse_duration > 0.01:       # timeout
                return -1

        if pulse_duration is None:
            return -1

        distance = pulse_duration * 17150
        distance = round(distance, 2)
        return distance


    def obstacle_detection_handler(self):
        distance = self.get_ultrasound_distance()
        now = time.time()
        if distance < 0:
            if now - self.last_time > 0.5:
                self.publishRange(self.pub_ultrasound, ULTRASOUND, float("Inf"))
            else:
                self.publishRange(self.pub_ultrasound, ULTRASOUND, self.last_distance)
        else:
            self.publishRange(self.pub_ultrasound, ULTRASOUND, distance)
            self.last_distance = distance
            self.last_time = now

    def startTOFSensor(self):
        self.loginfo("establishing connection with tof sensor")
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.tof_sensor = adafruit_vl6180x.VL6180X(i2c)
            self.tof_sensor_online = True
        except (ValueError, OSError) as e:
            self.loginfo('Could not connect to TOF sensor')
            print(e)
            self.tof_sensor_online = False

    def loginfo(self, s):
        self.get_logger().info(s)


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
