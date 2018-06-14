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

from abc import ABCMeta, abstractmethod
from collections import namedtuple

FAMILY_LINE_DETECTOR = 'line_detector'

Detections = namedtuple('Detections', 
                        ['lines','normals','area','centers'])


class LineDetectorInterface():
    __metaclass__ = ABCMeta

    @abstractmethod
    def setImage(self, bgr):
        pass

    @abstractmethod
    def detectLines(self, color):
        """ Returns a tuple of class Detections """


