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
   
    arr = [("pi_camera", "camera_node_sequence", True),
            ("dagu_car", "wheels_driver_node", True),
            ("joy", "joy_node", True)]

    for package, executable, required in arr:
        ld.add_process(
            cmd=[get_executable_path(package_name=package, executable_name=executable)],
            name=executable,
            # die if required, restart otherwise
            exit_handler=default_exit_handler if required else restart_exit_handler,
        ) 

    package, executable,required = "joy_mapper", "joy_mapper_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
            "--publish_topic", "/joy_mapper_node/car_cmd"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    package, executable,required = "dagu_car", "inverse_kinematics_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
            "--subscribe_topic", "/car_cmd",
            "--publish_topic", "/wheels_cmd"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    package, executable, required = "object_classifier", "object_classification_node", True
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name=executable),
            "--subscribe_topic", "/image/compressed",
            "--publish_topic", "/object_classifier/output"],
        name=executable,
        # die if required, restart otherwise
        exit_handler=default_exit_handler if required else restart_exit_handler
    )
    return ld
