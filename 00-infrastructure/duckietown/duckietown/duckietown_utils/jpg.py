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
