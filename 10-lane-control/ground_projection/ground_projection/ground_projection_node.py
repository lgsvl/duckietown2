# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import sys
import cv2
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
#import pickle

#from ground_projection_srv import EstimateHomography, GetGroundCoord, GetImageCoord
from duckietown_msgs.msg import Pixel, Vector2D, Segment, SegmentList
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import numpy as np
from ground_projection.ground_projection_include.GroundProjection import GroundProjection


class GroundProjectionNode(Node):
    def __init__(self):
        self.node_name = 'ground_projection_node'
        super().__init__(self.node_name)

        self.bridge = CvBridge()
        self.active = True

        self.robot_name = ""
        self.gp = GroundProjection(self.robot_name)

        camera_info_topic = "camera_info"
        self.loginfo("camera info topic is " + camera_info_topic)
        #self.loginfo("waiting for camera info")

        #filepath = "/home/brian/ros2_ws/install/include/ground_projection/birdbot0.yaml"
        filepath = os.path.abspath(os.path.join(os.getcwd(), 'install/include/ground_projection/birdbot5.yaml'))
        camera_info = self.load_camera_info(filepath)
        self.gp.initialize_pinhole_camera_model(camera_info)

        #self.gp.initialize_pinhole_camera_model(camera_info)

        self.gp.robot_name = self.robot_name
        #self.gp.rectified_input = get_param(afklajflkajfl)
        self.image_channel_name = "image_raw"

        self.pub_lineseglist_ = self.create_publisher(SegmentList, "segment_list_out")
        self.sub_lineseglist_ = self.create_subscription(SegmentList, "segment_list_in", self.lineseglist_cb)

        #self.service_homog_ = self.create_service(EstimateHomography, "estimate_homography", self.estimate_homography_cb)

        # skip services that retrieve parameters for now


    """
    def rectifyImage(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="mono8")
        except CvBridgeError as e:
            print("Could not convert to CV image ({0})".format(e))
        return gp.rectify(cv_image)
    """

    def lineseglist_cb(self, seglist_msg):
        message_time = seglist_msg.header.stamp.sec + seglist_msg.header.stamp.nanosec*1e-9
        current_time = time.time()
        delay = current_time - message_time
        
        #fname = "/dev/shm/segment" + str(message_time) 
        #seglist_msg.segments = pickle.load(open(fname, "rb"))
        #os.unlink(fname)

        #print("message time: " + str(message_time))
        #print("current time: " + str(current_time))
        #print("delay: " + str(delay), "# segments: ", len(seglist_msg.segments))

        seglist_out = SegmentList()
        seglist_out.header = seglist_msg.header
        for received_segment in seglist_msg.segments:
            new_segment = Segment()
            new_segment.points[0] = self.gp.vector2ground(received_segment.pixels_normalized[0])
            new_segment.points[1] = self.gp.vector2ground(received_segment.pixels_normalized[1])
            new_segment.color = received_segment.color
            # TODO what about normal and points
            seglist_out.segments.append(new_segment)
        self.pub_lineseglist_.publish(seglist_out)

    def loginfo(self, s):
        self.get_logger().info('[%s %s]' % (self.node_name, s))

    def load_camera_info(self, filename):
        import yaml
        try:
            f = open(filename, 'r')
        except IOError as err:
            print("Could not open file, IO Error({0})".format(err))
            return

        calib_data = yaml.safe_load(f)
        cam_info = CameraInfo()

        # in ROS2 all the cameraInfo attributes are lower case
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.k = calib_data['camera_matrix']['data']
        cam_info.d = calib_data['distortion_coefficients']['data']
        cam_info.r = calib_data['rectification_matrix']['data']
        cam_info.p = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = GroundProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
