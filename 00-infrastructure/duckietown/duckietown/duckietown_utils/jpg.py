# Copyright (c) 2018 LG Electronics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

"""
    The many options to convert JPG into data.
"""

import os

def jpg_from_image_cv(image):
    import cv2
    return cv2.imencode('.jpg', image)[1].tostring()

def image_cv_from_jpg(data):
    import cv2
    import numpy as np
    #s = np.fromstring(data, np.uint8)
    s = np.frombuffer(data, np.uint8)
    image_cv = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if image_cv is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return image_cv
