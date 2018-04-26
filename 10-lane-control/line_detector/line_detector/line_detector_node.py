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

import os
import sys
import cv2
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node

#from anti_instagram.AntiInstagram import AntiInstagram
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import AntiInstagramTransform, BoolStamped, Segment, SegmentList, Vector2D
from duckietown.duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker

from line_detector.line_detector_include.line_detector1 import LineDetectorHSV
from line_detector.line_detector_include.line_detector_plot import color_segment, drawLines

import pickle

class LineDetectorNode(Node):
    def __init__(self):
        self.node_name = 'line_detector_node'
        super().__init__(self.node_name)

        self.thread_lock = threading.Lock()

        self.bridge = CvBridge()
        self.active = True

        #self.stats = Stats()

        # Only be verbose every 10 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0

        # color correction
        #self.ai = AntiInstagram()
        # these will be added if it becomes verbose
        self.pub_edge = None
        self.pub_colorSegment = None

        self.verbose = False

        self.detector = None
        self.detector_config = None

        self.pub_lines = self.create_publisher(SegmentList, "segment_list_in")
        self.pub_image = self.create_publisher(Image, "image_with_lines")

        self.sub_image = self.create_subscription(CompressedImage, "image/compressed", self.cbImage)
        self.sub_transform = self.create_subscription(AntiInstagramTransform, "transform", self.cbTransform)
        self.sub_switch = self.create_subscription(BoolStamped, "switch", self.cbSwitch)

        self.loginfo("[%s] Initialized (verbose = %s)." %(self.node_name, self.verbose))

        # NEED to WAIT SOME AMOUNT OF TIME (2secs) before calling updateParams
        #filepath = "/home/brian/ros2_ws/install/include/line_detector/default.yaml"
        #filepath = os.path.abspath('include/line_detector/default.yaml')
        filepath = os.path.abspath(os.path.join(os.getcwd(), 'install/include/line_detector/default.yaml'))
        self.loadConfig(filepath)
        self.updateParams(None)

    def loadConfig(self,filename):
        """ custom function because we're not dealing with parameters in ROS2 python right now. 
        load configuration parameters manually."""
        import yaml
        try:
            f = open(filename, 'r')
        except IOError as err:
            print("Could not open file, IO error({0})".format(err))
            return

        config = yaml.safe_load(f)
        
        self.image_size = config['img_size'] # list of two elements
        self.top_cutoff = config['top_cutoff'] # integer

        self.detector_config = config['detector'][1]['configuration']
        

    def updateParams(self, _event):
        ############### TO DO #######################
        old_verbose = self.verbose
        # set self.verbose from value retrieved from parameter "verbose"
        if self.verbose != old_verbose:
            self.loginfo("Verbose is now %r" % self.verbose)

        # retrieve image size from parameters
        # retrieve top cutoff from parameters

        # not using duckietown instantiate()
        self.detector = LineDetectorHSV(self.detector_config)

        if self.verbose and self.pub_edge is None:
            self.pub_edge = self.create_publisher(Image, "edge")
            self.pub_colorSegment = self.create_publisher(Image, "colorSegment")

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def cbImage(self, image_msg):
        # self.stats.received()
        if not self.active:
            return
        thread = threading.Thread(target=self.processImage,args=(image_msg,), daemon=True)
        thread.start()
        # Returns rightaway
        
    def cbTransform(self, transform_msg):
        self.ai.shift = transform_msg.s[0:3]
        self.ai.scale = transform_msg.s[3:6]
        
        self.loginfo("AntiInstagram transform received")

    def loginfo(self, s):
        self.get_logger().info('%s' % (s))

    def intermittent_log_now(self):
        return (self.intermittent_counter % self.intermittent_interval == 1)

    def intermittent_log(self, s):
        if not self.intermittent_log_now():
            return
        self.loginfo('%3d:%s' % (self.intermittent_counter, s))

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            #self.stats.skipped()
            return

        try:
            self.processImage_(image_msg)
            #image_msg.data = list(open("/dev/shm/a.jpg", "rb").read())
            message_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec*1e-9
            current_time = time.time()
            delay = current_time - message_time
            print("message time: " + str(message_time))
            print("current time: " + str(current_time))
            print("delay: " + str(delay))

        finally:
            self.thread_lock.release()

    def processImage_(self, image_msg):
        # self.stats.processed()
        #if self.intermittent_log_now():
            #self.intermittent_log(self.stats.info())
            #self.stats.reset()
        # tk = TimeKeeper(image_msg)
        self.intermittent_counter += 1

        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return
        # tk.completed('decoded')

        hei_original,wid_original = image_cv.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image_cv = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
                                   interpolation=cv2.INTER_NEAREST)
        image_cv = image_cv[self.top_cutoff:,:,:]

        #tk.completed('resized')
        #image_cv_corr = self.ai.applyTransform(image_cv)
        #image_cv_corr = cv2.convertScaleAbs(image_cv_corr)
        image_cv_corr = cv2.convertScaleAbs(image_cv)

        # Set the image to be detected
        self.detector.setImage(image_cv_corr)
        # Detect lines and normals
        white = self.detector.detectLines('white')
        yellow = self.detector.detectLines('yellow')
        red = self.detector.detectLines('red')

        white = white[0]
        yellow = yellow[0]
        red = red[0]

        # tk.completed('detected')

        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Convert to normalized pixel coordinates, and add segments to segmentList
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))
        if len(white.lines) > 0:
            lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
        if len(yellow.lines) > 0:
            lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
        if len(red.lines) > 0:
            lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))
        
        self.intermittent_log('# segments: white %3d yellow %3d red %3d' % (len(white.lines),
                len(yellow.lines), len(red.lines)))
       
        #tk.completed('prepared')
        # Publish segmentList
        #message_time = segmentList.header.stamp.sec + segmentList.header.stamp.nanosec*1e-9
        #fname = "/dev/shm/segment" + str(message_time)
        #open(fname, "wb").write(pickle.dumps(segmentList.segments, pickle.HIGHEST_PROTOCOL))
        #segmentList.segments = []
        self.pub_lines.publish(segmentList)
        self.loginfo("published line segments")
        #tk.completed('--pub_lines--')

        if self.verbose:
            # Draw lines and normals
            image_with_lines = np.copy(image_cv_corr)
            drawLines(image_with_lines, white.lines, (0, 0, 0))
            drawLines(image_with_lines, yellow.lines, (255, 0, 0))
            drawLines(image_with_lines, red.lines, (0, 255, 0))
            #tk.completed('drawn')
            # Publish the frame with lines
            image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
            image_msg_out.header.stamp = image_msg.header.stamp
            self.pub_image.publish(image_msg_out)
            #tk.completed('pub_image')
#         if self.verbose:
            colorSegment = color_segment(white.area, red.area, yellow.area) 
            edge_msg_out = self.bridge.cv2_to_imgmsg(self.detector.edges, "mono8")
            colorSegment_msg_out = self.bridge.cv2_to_imgmsg(colorSegment, "bgr8")
            self.pub_edge.publish(edge_msg_out)
            self.pub_colorSegment.publish(colorSegment_msg_out)
            #tk.completed('pub_edge/pub_segment')
        #self.intermittent_log(tk.getall())


    def toSegmentMsg(self,lines, normals, color):
        segmentMsgList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
             
            segmentMsgList.append(segment)
        return segmentMsgList



def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = LineDetectorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

