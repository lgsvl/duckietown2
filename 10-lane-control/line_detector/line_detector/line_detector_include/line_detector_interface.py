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


