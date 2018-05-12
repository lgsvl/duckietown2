#!/usr/bin/env python

import sys
import threading
import time
import socket
import struct
import argparse

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Joy
from duckietown.duckietown_utils.jpg import image_cv_from_jpg
from duckietown_msgs.msg import Twist2DStamped

IMAGE_DIM = (160, 120)


class DLLaneFollowingNode(Node):
    def __init__(self, args):
        self.node_name = 'dl_lane_following_node'
        super().__init__(self.node_name)

        self.args = args        

        # thread lock
        self.thread_lock = threading.Lock()

        self.sock = None
        self.state = 1
        
        self.max_speed = 0.2
        self.min_speed = 0.1
        self.omega_threshold = 2.5
        
        self.host = self.args.host
        self.port = self.args.port
        self.speed = self.args.speed
        self.omega_gain = self.args.omega_gain

        self.addr = (self.host, self.port)
        
        # Subscriber
        self.sub_image = self.create_subscription(CompressedImage, "/simulator/camera_node/image/compressed", self.callback)
        #self.sub_joy_btn = self.create_subscription(Joy, '/joy', self.callback_joy_btn)
        
        # Publisher
        self.pub_car_cmd = self.create_publisher(Twist2DStamped, self.args.publish_topic)
    
        self.loginfo("Initialized")

    def callback(self, image_msg):
        if self.state == -1:
            return

        # start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    """
    def callback_joy_btn(self, joy_msg):
        if joy_msg.buttons[5] == 1:  # RB joypad botton
            self.state *= -1
            if self.state == 1:
                self.loginfo('[{}] Start lane following'.format(self.node_name))
            if self.state == -1:
                self.loginfo('[{}] Stop lane following'.format(self.node_name))
    """

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
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
        
        buf = cv2.imencode(".jpg", img)[1].tostring()

        # create socket connection
        if self.sock is None:
            self.loginfo("Connecting to %s" % str(self.addr))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            try:
                self.sock.connect(self.addr)
            except socket.error as e:
                print("Exception socket error: %s" % e)
                self.sock.close()
                self.sock = None
                return
            self.sock.settimeout(1.0)
            self.loginfo("Connected")

        data = b''
        try:
            self.sock.send(struct.pack("<I", len(buf)) + buf)
            # self.sock.send(struct.pack("<I", len(buf)))
            # while buf:
                # self.sock.send(buf[:4])
                # buf = buf[4:]
            while len(data) < 4:
                data += self.sock.recv(4)
        except socket.error as e:
            self.loginfo("Disconnected from server")
            print("Socket send error: %s" % e)
            self.sock.close()
            self.sock = None
            return

        predicted_omega = struct.unpack_from("<f", data)[0]

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
    parser.add_argument("--host",
                        type=str,
                        default="localhost",
                        help="ip address of host running machine learning server")
    parser.add_argument("--port",
                        type=int,
                        default=12322,
                        help="port number of host")
    parser.add_argument("--speed",
                        type=float,
                        default=0.2,
                        help="wheel speed velocity gain")
    parser.add_argument("--omega_gain",
                        type=float,
                        default=1.0,
                        help="multiplier for trim vehicle turning rate")
    parser.add_argument("--publish_topic",
                        type=str,
                        default="/joy_mapper_node/car_cmd",
                        help="name of topic to publish car command to")
    args = parser.parse_args()
    node = DLLaneFollowingNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
