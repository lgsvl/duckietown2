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


def get_cmd(package, executable, required, *args):
    cmd = [get_executable_path(package_name=package, executable_name=executable)]
    for arg in args:
        cmd.append(arg)

    return cmd


def launch(launch_descriptor, argv):
    ld = launch_descriptor

    arg_map = {
        'camera_topic': '/image/compressed',
        'object_topic': '/object_classifier/output',
        'verbose': 'True',
        'min_score_threshold': '0.1',
    }

    for arg in argv:
        key, value = arg.split(':=')
        if key in arg_map:
            arg_map[key] = value

    nodes = [
        ("pi_camera", "camera_node_sequence", True),
        ("object_classifier", "object_classification_node", True, "--camera_topic", arg_map["camera_topic"], "--object_topic", arg_map["object_topic"], "--verbose", arg_map["verbose"], "--min_score_threshold", arg_map["min_score_threshold"]), 
    ]

    for node in nodes: 
        package, executable, required = node[:3]
        cmd = get_cmd(*node)
        ld.add_process(
            cmd=cmd,
            name=executable,
            exit_handler=default_exit_handler if required else restart_exit_handler,
        )

    return ld

