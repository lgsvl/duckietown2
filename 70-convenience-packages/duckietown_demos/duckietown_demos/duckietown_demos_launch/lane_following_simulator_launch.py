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

from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    arr = [("ground_projection", "ground_projection_node", True),
            ("lane_filter", "lane_filter_node", True),
            ("joy", "joy_node", True)]

    for package, executable, required in arr:
        ld.add_process(
            cmd=[get_executable_path(package_name=package, executable_name=executable)],
            name=executable,
            # die if required, restart otherwise
            exit_handler=default_exit_handler if required else restart_exit_handler,
        )

    package, executable,required = "line_detector", "line_detector_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
             "--subscribe_topic", "/simulator/camera_node/image/compressed"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    package, executable,required = "dagu_car", "inverse_kinematics_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
             "--publish_topic", "/simulator/wheels_driver_node/wheels_cmd"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    package, executable,required = "lane_control", "lane_controller_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
             "--gain", "0.4",
             "--publish_topic", "/lane_controller_node/car_cmd"],
        name="lane_controller_node",
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    ld.add_process(
        cmd=[get_executable_path(package_name="joy_mapper", executable_name="joy_mapper_node"),
             "--publish_topic", "/joy_mapper_node/car_cmd"],
        name="joy_mapper_node",
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler,
    )
    package, executable, required = "dagu_car", "car_cmd_switch_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
            "--subscribe_topic1", "/joy_mapper_node/car_cmd",
            "--subscribe_topic2", "/lane_controller_node/car_cmd",
            "--subscribe_topic_switch", "/joystick_override",
            "--publish_topic", "/car_cmd"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler,
    )

    return ld
