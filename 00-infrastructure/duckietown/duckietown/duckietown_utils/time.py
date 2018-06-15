# Copyright (c) 2018 LG Electronics, Inc.
#
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
