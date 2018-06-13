
import rclpy
from rclpy.node import Node

import yaml
import _thread as thread
import io
import sys
import time

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo
#from duckietown_utils import get_duckiefleet_root
from duckietown_msgs.msg import BoolStamped

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading

def timeStamp():
    current_time = time.time()

    t = Time()
    t.sec = int(current_time)
    t.nanosec = int(current_time%1 * 1E9)

    return t

class WebHandler(BaseHTTPRequestHandler):
  def do_GET(self):
    boundary = "--boundarydonotcross"
    self.send_response(200)
    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0")
    self.send_header("Connection", "close")
    self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=%s" % boundary)
    self.send_header("Expires", "Mon, 3 Jan 2000 12:34:56 GMT")
    self.send_header("Pragma", "no-cache")
    self.end_headers()
    try:
      while True:
        self.server.camera.lock.acquire()
        self.server.camera.cv.wait()
        image = self.server.camera.image
        self.server.camera.lock.release()

        self.wfile.write(bytes(boundary, "utf-8"))
        self.end_headers()

        self.send_header("X-Timestamp", time.time())
        self.send_header("Content-Length", len(image))
        self.send_header("Content-Type", "image/jpeg")
        self.end_headers()

        self.wfile.write(image)
        self.end_headers()
    except:
      pass

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
  pass

def web_serve(camera):
  try:
    httpd = ThreadedHTTPServer(('', 8001), WebHandler)
    httpd.camera = camera
    httpd.serve_forever()
  except:
    pass


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.node_name = self.get_name()
        self.log = self.get_logger()

        self.log.info("Initializing......")

        # TODO:ROSv2: hardcoded values
        self.framerate_high = 30.0 # self.setupParam("~framerate_high",30.0)
        self.framerate_low = 15.0 # self.setupParam("~framerate_low",15.0)
        self.res_w = 640 # self.setupParam("~res_w",640)
        self.res_h = 480 # self.setupParam("~res_h",480)
        self.simulator = False # rospy.get_param("~use_simulator_cam")

        self.image_msg = CompressedImage()

        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)

        thread.start_new_thread(web_serve, (self,))

        # Setup PiCamera

        self.framerate = self.framerate_high # default to high

        if self.simulator:
            self.camera = None
        else:
            from picamera import PiCamera
            self.camera = PiCamera()
            self.camera.framerate = self.framerate
            self.camera.resolution = (self.res_w, self.res_h)

        # For intrinsic calibration
        # TODO:ROSv2:
        # self.cali_file_folder = get_duckiefleet_root() + "/calibrations/camera_intrinsic/"
    
        # TODO:ROSv2: rospy.get_namespace().strip('/') + "/camera_optical_frame"
        self.frame_id = "camera_optical_frame"

        self.has_published = False
        self.pub_img = self.create_publisher(CompressedImage, "image/compressed") # queue_size=1
        self.sub_switch_high = self.create_subscription(BoolStamped, "framerate_high_switch", self.cbSwitchHigh) # queue_size=1

        # Create service (for camera_calibration)
        self.srv_set_camera_info = self.create_service(SetCameraInfo, "set_camera_info", self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()

        self.is_shutdown = False
        self.update_framerate = False

        # Setup timer
        self.log.info("Initialized.")


    def cbSwitchHigh(self, switch_msg):
        self.log.info(switch_msg)
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True
 

    def startCapturing(self):
        self.log.info("Start capturing.")
        while not self.is_shutdown:
            gen = self.grabAndPublish(self.stream)
            try:
                self.camera.capture_sequence(gen, "jpeg", use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            self.log.info("updating framerate")
            self.camera.framerate = self.framerate
            self.update_framerate = False

        self.camera.close()
        self.log.info("Capture Ended.")


    def grabAndPublish(self, stream):
        while not self.update_framerate and not self.is_shutdown:
            yield stream
            # Construct image_msg
            # Grab image from stream
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            
            image_msg.data = stream_data

            self.lock.acquire()
            self.image = stream_data
            self.cv.notify_all()
            self.lock.release()

            #image_msg.data = []
            #open("/dev/shm/a.jpg", "wb").write(bytes(stream_data))

            image_msg.header.stamp = timeStamp()
            image_msg.header.frame_id = self.frame_id
            self.pub_img.publish(image_msg)
                        
            # Clear stream
            stream.seek(0)
            stream.truncate()
            
            if not self.has_published:
                self.log.info("Published the first image.")
                self.has_published = True

            time.sleep(0.001)

    #def setupParam(self,param_name,default_value):
    #    value = rospy.get_param(param_name,default_value)
    #    rospy.set_param(param_name,value) #Write to parameter server for transparancy
    #    rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
    #    return value

    def destroy_node(self):
        self.log.info("Closing camera.")
        self.is_shutdown = True
        self.log.info("Shutdown.")
        super().destroy_node()


    def cbSrvSetCameraInfo(self, req):
        # TODO: save req.camera_info to yaml file
        self.log.info("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + self.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfo.Response()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename #TODO file name
        return response


    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        self.log.info("[saveCameraInfo] filename: %s" %(filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': self.get_name().strip("/"), #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
        
        self.log.info("[saveCameraInfo] calib %s" % calib)

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False


def main(args = None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = CameraNode()
    thread.start_new_thread(node.startCapturing, ())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
