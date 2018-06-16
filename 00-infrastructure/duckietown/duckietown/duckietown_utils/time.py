# Copyright (c) 2018 LG Electronics, Inc.
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

"""
    Utility functions for time related calculations
"""

import time
from builtin_interfaces.msg import Time

def get_current_time_msg():
    current_time = time.time()
    time_msg = Time()
    time_msg.sec = int(current_time)
    time_msg.nanosec = int(current_time%1 * 1E9)
    return time_msg
