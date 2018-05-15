#!/usr/bin/env python

import sys
import os
import threading
import time
import argparse

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from duckietown.duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import CompressedImage, Image, Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from mvnc import mvncapi as mvnc


IMAGE_DIM = (160, 120)
BASE_PATH = os.path.dirname(os.path.realpath(__file__))
GRAPH_NAME = 'caffe_model_2.graph'


class DLLaneFollowingNCSNode(Node):
    def __init__(self, args):
        self.node_name = 'dl_lane_following_ncs_node'
        super().__init__(self.node_name)

        self.args = args
        
        # thread lock
        self.thread_lock = threading.Lock()

        self.sock = None
        self.state = 1
        
        self.max_speed = 0.2
        self.min_speed = 0.1
        self.omega_threshold = 2.5
        
        self.speed = self.args.speed
        self.omega_gain = self.args.omega_gain

        devices = mvnc.EnumerateDevices()
        device = mvnc.Device(devices[0])
        device.OpenDevice()

        with open('{}/host/model/caffe/{}'.format(BASE_PATH, GRAPH_NAME), mode='rb') as f:
            graph_in_memory = f.read()

        self.graph = device.AllocateGraph(graph_in_memory)
        # self.graph.SetGraphOption(mvnc.GlobalOption.LOG_LEVEL, 2)
        self.loginfo('[{}] Graph allocated: {}'.format(self.node_name, GRAPH_NAME))

        self.sub_image = self.create_subscription(CompressedImage, self.args.subscribe_topic, self.callback)
        self.sub_joy_btn = self.create_subscription(BoolStamped, self.args.joystick_override, self.joystick_override_callback)
        self.pub_car_cmd = self.create_publisher(Twist2DStamped, self.args.publish_topic)

    def callback(self, image_msg):
        if self.state == 1:
            return
        # start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def joystick_override_callback(self, joystick_override_msg):
        self.loginfo("Switching to joystick mode: " + str(joystick_override_msg.data))
        self.state = 1 if joystick_override_msg.data else -1

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
            #message_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec*1e-9
            #current_time = time.time()
            #delay = current_time - message_time
            #print("message time: " + str(message_time))
            #print("current time: " + str(current_time))
            #print("delay: "  + str(delay))
        finally:
            # release the thread lock
            self.thread_lock.release()

    def processImage_(self, image_msg):
        t1 = time.time()
        # decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        # import image for classification
        img = cv2.resize(image_cv, IMAGE_DIM, interpolation=cv2.INTER_NEAREST)
        img = img[50:, :, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
        
        self.graph.LoadTensor(img.astype(np.float16), 'user_object')
        preds, _ = self.graph.GetResult()
        
        predicted_omega = preds[0]

        # set car cmd through ros message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = image_msg.header
        car_control_msg.v = self.speed
        # car_control_msg.v = self.normalize_speed(predicted_omega, self.omega_threshold, self.min_speed, self.max_speed)
        car_control_msg.omega = predicted_omega * self.omega_gain
        t2 = time.time()

        #print('Time: %.3f Speed: %.3f Omega: %.3f' % ((t2 - t1), car_control_msg.v, car_control_msg.omega))
        
        # publish the control command
        self.publishCmd(car_control_msg)

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

    def normalize_speed(self, w, w_max, v_min, v_max):
        w_min = 0.0
        v_min, v_max = -v_max, -v_min
        v = abs((v_max - v_min) / (w_max - w_min) * (abs(w) - w_max) + v_max)
        if v < v_min:
            v = v_min

        return v

    def loginfo(self, s):
        self.get_logger().info(s)


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed",
                        type=float,
                        default=0.2,
                        help="wheel speed velocity gain")
    parser.add_argument("--omega_gain",
                        type=float,
                        default=3.0,
                        help="multiplier for trim vehicle turning rate")
    parser.add_argument("--subscribe_topic",
                        type=str,
                        default="/simulator/camera_node/image/compressed",
                        help="name of topic to subscribe to for camera images")
    parser.add_argument("--publish_topic",
                        type=str,
                        default="/simulator/joy_mapper_node/car_cmd",
                        help="name of topic to publish car command to")
    parser.add_argument("--joystick_override",
                        type=str,
                        default="/joystick_override",
                        help="name of topic to subscribe to for joystick override signal")
    args = parser.parse_args()
    node = DLLaneFollowingNCSNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 

if __name__ == '__main__':
    rospy.init_node('dl_lane_following', anonymous=False)
    dl_lane_following = dl_lane_following()
    rospy.spin()
