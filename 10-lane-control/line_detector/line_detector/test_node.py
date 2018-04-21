
import sys

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import AntiInstagramTransform, BoolStamped, Segment, SegmentList, Vector2D
from duckietown.duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker

from line_detector.line_detector_include.line_detector1 import LineDetectorHSV
from line_detector.line_detector_include.line_detector_plot import color_segment, drawLines

class TestNode(Node):
    def __init__(self):
        self.node_name = 'test_node'
        super().__init__(self.node_name)
        
        self.sub = self.create_subscription(SegmentList, "segment_list", self.callback)

    def callback(self, msg):
        segments = msg.segments
        for segment in segments:
            self.loginfo(segment.pixels_normalized.x + ", " + segment.pixels_normalized.y)
            self.loginfo(segment.normal)
            self.loginfo(segment.points)

    def loginfo(self, s):
        self.get_logger().info('%s' % (s)) 


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
