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
