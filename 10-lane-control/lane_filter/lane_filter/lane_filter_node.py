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

import sys
import os
import time

import rclpy
from rclpy.node import Node

#from anti_instagram.AntiInstagram import AntiInstagram
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, Segment, SegmentList, Pixel, LanePose, Twist2DStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from lane_filter.lane_filter_include.lane_filter import LaneFilterHistogram

class LaneFilterNode(Node):
    def __init__(self):
        self.node_name = 'lane_filter_node'
        super().__init__(self.node_name)

        self.active = True

        self.filter = None
        self.filter_config = None        

        self.t_last_update = time.time()        # replace later with rclpy get_time() is available
        self.velocity = Twist2DStamped()
        
        self.sub = self.create_subscription(SegmentList, "segment_list_out", self.processSegments)
        self.sub_switch = self.create_subscription(BoolStamped, "switch", self.cbSwitch)
        self.sub_velocity = self.create_subscription(Twist2DStamped, "car_cmd", self.updateVelocity)

        self.pub_lane_pose = self.create_publisher(LanePose, "lane_pose")
        self.pub_belief_img = self.create_publisher(Image, "belief_img")
        self.pub_ml_img = self.create_publisher(Image, "ml_img")
        self.pub_entropy = self.create_publisher(Float32, "entropy")
        self.pub_in_lane = self.create_publisher(BoolStamped, "in_lane")

        # config file with filter parameters
        filepath = os.path.abspath(os.path.join(os.getcwd(), 'install/include/lane_filter/default.yaml'))    
        self.loadConfig(filepath)
        self.updateParams(None)
        
    def loadConfig(self,filename):
        import yaml
        try:
            f = open(filename, 'r')
        except IOError as err:
            print("Could not open file, IO error ({0})".format(err))
            return
        config = yaml.safe_load(f)
        self.filter_config  = config['filter'][1]['configuration'] 
        print(self.filter_config)
               
    def updateParams(self, event):
        """ set type of Bayes filter used for estimating pose in lane"""
        self.filter = LaneFilterHistogram(self.filter_config)

    def cbSwitch(self, switch_msg):
        #self.active = switch_msg.data
        # no switch for now
        self.active = True

    def processSegments(self, segment_list_msg):
        if not self.active:
            return

        current_time = time.time()
        self.filter.predict(dt=current_time - self.t_last_update, v=self.velocity.v, w=self.velocity.omega)
        self.t_last_update = current_time

        ml = self.filter.update(segment_list_msg.segments)
        if ml is not None:
            ml_img = self.getDistributionImage(ml,segment_list_msg.header.stamp)
            self.pub_ml_img.publish(ml_img)

        [d_max, phi_max] = self.filter.getEstimate()
        max_val = self.filter.getMax()
        in_lane = bool(max_val > self.filter.min_max)

        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        lanePose.status = lanePose.NORMAL
    
        # publish the belief image
        belief_img = self.getDistributionImage(self.filter.belief,segment_list_msg.header.stamp)

        current_time_print = time.time()
        message_time = segment_list_msg.header.stamp.sec + segment_list_msg.header.stamp.nanosec*1e-9
        delay = current_time_print - message_time
        #print("wheels_cmd timestamp: " + str(segment_list_msg.header.stamp))
        #print("current time: " + str(current_time_print))
        #print("delay: " + str(delay))
        

        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg) 

    def getDistributionImage(self,mat,stamp):
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg((255*mat).astype('uint8'), "mono8")
        img.header.stamp = stamp
        return img
        
    def updateVelocity(self,twist_msg):
        self.velocity = twist_msg
    
    def loginfo(self, s):
        self.get_logger().info('%s' % (s))


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = LaneFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
