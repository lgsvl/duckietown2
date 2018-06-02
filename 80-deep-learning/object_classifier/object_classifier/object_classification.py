#!/usr/bin/env python
import os
import sys
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import threading
import numpy as np
import cv2
from mvnc import mvncapi as mvnc
from duckietown_msgs.msg import ClassifiedObject


IMAGE_DIM = (299, 299)
BASE_PATH = os.path.dirname(os.path.realpath(__file__))
GRAPH_FILE = 'graph'
CATEGORY_FILE = 'categories.txt'


class object_classifier(Node):
    def __init__(self, args):
        super().__init__('object_classifier')

        self.args = args

        self.sub_image = self.create_subscription(CompressedImage, '/image/compressed', self.call_back)
        self.pub_obj = self.create_publisher(ClassifiedObject, '/object_classifier/output')

        self.verbose = self.args.verbose 
        self.min_score_threshold = self.args.min_score_threshold

        self.prev_category = None
        self.categories = []
        with open('{}/inception-v4/{}'.format(BASE_PATH, CATEGORY_FILE), 'r') as f:
            for line in f:
                cat = line.split('\n')[0]
                if cat != 'classes':
                    self.categories.append(cat)
            f.close()

        devices = mvnc.EnumerateDevices()
        if len(devices) == 0:
            print('No devices found')
            return

        device = mvnc.Device(devices[0])
        device.OpenDevice()
        
        with open('{}/inception-v4/{}'.format(BASE_PATH, GRAPH_FILE), mode='rb') as f:
            graph_in_memory = f.read()

        self.graph = device.AllocateGraph(graph_in_memory)
        self.thread_lock = threading.Lock()

    def call_back(self, data):
        thread = threading.Thread(target=self.processImage, args=(data,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, data):
        if not self.thread_lock.acquire(False):
            return

        try:
            self.processImage_(data)
        finally:
            self.thread_lock.release()

    def processImage_(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv_image = self.resize_image(cv_image, IMAGE_DIM)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image = self.normalize_image(cv_image)

        self.graph.LoadTensor(cv_image.astype(np.float16), 'user object')
        output, _ = self.graph.GetResult()

        top_index = output.argsort()[::-1][0]
        detected_obj = {
            'label': self.categories[top_index],
            'score': output[top_index],
            'index': top_index,
        }

        if self.verbose == 'True' or self.verbose == 'true':
            print(detected_obj)

        if detected_obj['score'] > self.min_score_threshold:
            if self.prev_category != detected_obj['label']:
                self.prev_category = detected_obj['label']
                output = ClassifiedObject()
                output.header.stamp = data.header.stamp
                output.label = detected_obj['label']
                output.index = int(detected_obj['index'])
                output.score = float(detected_obj['score'])
                self.pub_obj.publish(output)
                print('Message published: {}, {:.1f}%, index {}'.format(detected_obj['label'], detected_obj['score'] * 100, detected_obj['index']))

    def resize_image(self, img, to_dim):
        dx, dy, dz = img.shape
        delta = float(abs(dy - dx))
        if dx > dy:
            img = img[int(0.5 * delta):dx - int(0.5 * delta), 0:dy]
        else:
            img = img[0:dx, int(0.5 * delta):dy - int(0.5 * delta)]

        resized_image = cv2.resize(img, to_dim, interpolation=cv2.INTER_NEAREST)

        return resized_image

    def normalize_image(self, img):
        normalized_image = cv2.normalize(img.astype('float'), None, -1.0, 1.0, cv2.NORM_MINMAX)

        return normalized_image


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--verbose",
        type=str,
        default="True",
    )
    parser.add_argument(
        "--min_score_threshold",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--camera_topic",
        type=str,
        default="/image/compressed",
    )
    parser.add_argument(
        "--object_topic",
        type=str,
        default="/object_classifier/output",
    )

    args = parser.parse_args()
    oc = object_classifier(args)
    rclpy.spin(oc)

    oc.destroy_node()
    rclpy.shutdown()

