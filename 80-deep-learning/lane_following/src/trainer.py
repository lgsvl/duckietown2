#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, Joy
import message_filters
import csv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils.jpg import image_cv_from_jpg
from datetime import datetime
import os
import errno


BASE_PATH = os.path.dirname(os.path.realpath(__file__))
CSV_PATH = u'{}/data'.format(BASE_PATH)
IMG_PATH = u'{}/data/img'.format(BASE_PATH)


class Trainer(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[{}] Initializing...".format(self.node_name))

		self.state = -1
		self.seq = 0

		# Create a directory if not exist
		self.mkdir_p(CSV_PATH)
		self.mkdir_p(IMG_PATH)
		
		# Subscriber
		self.sub_center_camera = message_filters.Subscriber('/simulator/camera_node/image/compressed', CompressedImage)
		self.sub_left_camera = message_filters.Subscriber('/simulator/camera_node/image_camL/compressed', CompressedImage)
		self.sub_right_camera = message_filters.Subscriber('/simulator/camera_node/image_camR/compressed', CompressedImage)
		self.sub_control = message_filters.Subscriber('/simulator/joy_mapper_node/car_cmd', Twist2DStamped)
		self.sub_joy_btn = rospy.Subscriber('/simulator/joy', Joy, self.callback_joy_btn)

		# Time Synchronizer
		self.ts = message_filters.ApproximateTimeSynchronizer([
			self.sub_center_camera, 
			self.sub_left_camera, 
			self.sub_right_camera, 
			self.sub_control
			], 10, 0.1)
		# self.ts = message_filters.TimeSynchronizer([self.sub_center_camera, self.sub_control], 10)		
		
		self.ts.registerCallback(self.callback)

		rospy.loginfo("[{}] Up and running...".format(self.node_name))

	def mkdir_p(self, path):
		try:
			os.makedirs(path)
		except OSError as exc:
			if exc.errno == errno.EEXIST and os.path.isdir(path):
				pass
			else:
				raise
		rospy.loginfo("[{}] Directory is ready: {}".format(self.node_name, path))

	def save_image(self, center_camera, left_camera, right_camera, img_id):
		center_image_cv = image_cv_from_jpg(center_camera.data)
		left_image_cv = image_cv_from_jpg(left_camera.data)
		right_image_cv = image_cv_from_jpg(right_camera.data)

		cv2.imwrite('{}/center-{}.jpg'.format(IMG_PATH, img_id), center_image_cv)
		cv2.imwrite('{}/left-{}.jpg'.format(IMG_PATH, img_id), left_image_cv)
		cv2.imwrite('{}/right-{}.jpg'.format(IMG_PATH, img_id), right_image_cv)

	def save_csv(self, control, img_id):
		with open('{}/training_data.csv'.format(CSV_PATH), 'ab') as f:
			writer = csv.writer(f, delimiter=',')
			writer.writerow([img_id, control.omega, control.v])

	def callback(self, center_camera, left_camera, right_camera, control):
		if self.state == -1:
			return 

		self.seq += 1
		rospy.loginfo("[{}] {} {} {} {}".format(self.node_name, self.seq, center_camera.format, control.v, control.omega))

		now = str(datetime.now().isoformat())

		self.save_image(center_camera, left_camera, right_camera, now)
		self.save_csv(control, now)

	def callback_joy_btn(self, joy_msg):
		if joy_msg.buttons[5] == 1:  # RB joypad botton
			self.state *= -1
			if self.state == 1:
				rospy.loginfo('[{}] Start recording'.format(self.node_name))
			if self.state == -1:
				rospy.loginfo('[{}] Stop recording'.format(self.node_name))


if __name__ == "__main__":
	rospy.init_node("trainer", anonymous=False)
	trainer = Trainer()
	rospy.spin()
