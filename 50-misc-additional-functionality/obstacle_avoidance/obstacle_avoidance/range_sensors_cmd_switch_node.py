# Copyright (c) 2018 LG Electronics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at

#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import time
import sys
import argparse

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

ULTRASOUND_DETECTION_THRESHOLD = 30 # distance threshold (cm) to register obstacle
OBSTACLE_TIME_THRESHOLD = 0.5		# time in seconds of hysteresis wait time
CLIFF_DIST_THRESHOLD = 75           # distance threshold (mm) for cliff detection

class RangeSensorCmdSwitchNode(Node):
    def __init__(self, args):
        self.node_name = "range_sensor_cmd_switch_node"
        super().__init__(self.node_name)

        self.args = args
        self.cliff_detected = False
        self.obstacle_detected = False
        self.ob_predicted_state = False
        self.ob_waiting_flag = False
        self.ob_time_start = time.time()


        self.pub_cmd = self.create_publisher(Twist2DStamped, self.args.publish_topic)
        self.pub_obstacle = self.create_publisher(BoolStamped, "/obstacle")

        self.sub_ultrasound = self.create_subscription(Range, "/sensor/ultrasound", self.cb_ultrasound)
        self.sub_tof = self.create_subscription(Range, "/sensor/tof", self.cb_tof)

        self.sub_joy_cmd = self.create_subscription(Twist2DStamped, self.args.subscribe_topic, self.joy_cmd_callback)

    def send_obstacle(self, stamp, detected):
        msg= BoolStamped()
        msg.header.stamp = stamp
        msg.data = detected
        self.pub_obstacle.publish(msg)

    def cb_tof(self, msg):
        is_cliff_detected = msg.range >= CLIFF_DIST_THRESHOLD
        if is_cliff_detected:   # cliff detected
            if not self.cliff_detected:
                self.loginfo('Cliff detected ahead!')
                if not self.obstacle_detected:
                    self.send_obstacle(msg.header.stamp, True)
        else:                   # no cliff detected
            if self.cliff_detected:
                self.loginfo('Safe now. No cliff ahead.')
                if not self.obstacle_detected:
                    self.send_obstacle(msg.header.stamp, False)
        self.cliff_detected = is_cliff_detected

    def cb_ultrasound(self, msg):
        distance = msg.range
        """
        if distance < ULTRASOUND_DETECTION_THRESHOLD:
            if not self.obstacle_detected:
                self.loginfo('Obstacle detected in {} m.'.format(distance))
                if not self.cliff_detected:
                    self.send_obstacle(msg.header.stamp, True)
        else:
            if self.obstacle_detected:
                self.loginfo('Safe now. No obstacles ahead.')
                if not self.cliff_detected:
                    self.send_obstacle(msg.header.stamp, False)

        self.obstacle_detected = distance < ULTRASOUND_DETECTION_THRESHOLD
        """
        obstacle_detected = distance < ULTRASOUND_DETECTION_THRESHOLD
        current_time = time.time()
        if obstacle_detected != self.obstacle_detected:
            self.ob_predicted_state = obstacle_detected
            if self.ob_waiting_flag == True:
                if current_time - self.ob_time_start > OBSTACLE_TIME_THRESHOLD:
                    self.loginfo('Obstacle status change detected. Obstacle detected: {}'.format(obstacle_detected))
                    self.send_obstacle(msg.header.stamp, obstacle_detected)
                    self.obstacle_detected = self.ob_predicted_state
                    self.ob_waiting_flag = False
            else:       # first time seeing change from sensor
                self.loginfo('Ultrasound detected change, waiting to stabilize')
                self.ob_waiting_flag = True
                self.ob_time_start = time.time()
        else:   # no change, or false alarm
            #self.loginfo('No change detected, or false alarm, going back to resting state')
            self.ob_waiting_flag = False
            self.ob_predicted_state = self.obstacle_detected

    def joy_cmd_callback(self, msg):
        if self.cliff_detected or self.obstacle_detected:
            if msg.v > 0.0:
              msg.v = 0.0
        self.pub_cmd.publish(msg)

    def loginfo(self, s):
        self.get_logger().info(s)


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--subscribe_topic",
                        type=str,
                        default="/car_cmd",
                        help="topic to subscribe to for car command")
    parser.add_argument("--publish_topic",
                        type=str,
                        default="/sensor/car_cmd",
                        help="name of topic to publish actual car command to")

    args = parser.parse_args()
    node = RangeSensorCmdSwitchNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
