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

from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_car_include.dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(Node):
    def __init__(self):
        self.node_name = 'wheels_driver_node'
        super().__init__(self.node_name)

        self.driver = DaguWheelsDriver()
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, 'wheels_cmd_executed')

        self.control_constant = 1.0

        self.sub_topic = self.create_subscription("wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd)
        self.sub_e_stop = self.create_subscription("emergency_stop", BoolStamped, self.cbEStop)

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.setWheelsSpeed(left=0.0,right=0.0)
            return
        self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()  
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd) 

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            self.get_logger().info("[%s] Emergency Stop Activated")
        else:
            self.get_logger().info("[%s] Emergency Stop Released")


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = WheelsDriverNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
