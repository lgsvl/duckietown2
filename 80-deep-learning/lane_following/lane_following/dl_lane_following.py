#!/usr/bin/env python
from __future__ import print_function
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image, Joy
from duckietown_utils.jpg import image_cv_from_jpg
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState
import cv2
import rospy
import threading
import time
import numpy as np
import socket
import numpy
import os
import sys
import struct


IMAGE_DIM = (160, 120)


class dl_lane_following(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # thread lock
        self.thread_lock = threading.Lock()

        self.host = rospy.get_param('~host')
        self.port = rospy.get_param('~port')
        self.speed = rospy.get_param('~speed')
        self.omega_gain = rospy.get_param('~omega_gain')

        self.addr = (self.host, self.port)
        
        # Subscriber
        self.sub_image = rospy.Subscriber("/simulator/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.sub_joy_btn = rospy.Subscriber('/simulator/joy', Joy, self.callback_joy_btn)
        
        # Publisher
        self.pub_car_cmd = rospy.Publisher("/simulator/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

        self.sock = None
        self.state = -1
        
        self.max_speed = 0.2
        self.min_speed = 0.1
        self.omega_threshold = 2.5

    def callback(self, image_msg):
        if self.state == -1:
            return

        # start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def callback_joy_btn(self, joy_msg):
        if joy_msg.buttons[5] == 1:  # RB joypad botton
            self.state *= -1
            if self.state == 1:
                rospy.loginfo('[{}] Start lane following'.format(self.node_name))
            if self.state == -1:
                rospy.loginfo('[{}] Stop lane following'.format(self.node_name))

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
            rospy.loginfo('Could not decode image: %s' % e)
            return

        # import image for classification
        img = cv2.resize(image_cv, IMAGE_DIM, interpolation=cv2.INTER_NEAREST)
        img = img[50:, :, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        buf = cv2.imencode(".jpg", img)[1].tostring()

        if self.sock is None:
            rospy.loginfo("Connecting to %s" % str(self.addr))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            try:
                self.sock.connect(self.addr)
            except:
            	self.sock.close()
                self.sock = None
                return
            self.sock.settimeout(1.0)
            rospy.loginfo("Connected")

        data = ""
        try:
            self.sock.send(struct.pack("<I", len(buf)) + buf)
            # self.sock.send(struct.pack("<I", len(buf)))
            # while buf:
                # self.sock.send(buf[:4])
                # buf = buf[4:]
            while len(data) < 4:
                data += self.sock.recv(4)
        except:
            rospy.loginfo("Disconnected from server")
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

        print('Time: %.3f Speed: %.3f Omega: %.3f' % ((t2 - t1), car_control_msg.v, car_control_msg.omega))

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


if __name__ == '__main__':
    rospy.init_node('dl_lane_following', anonymous=False)
    dl_lane_following = dl_lane_following()
    rospy.spin()
