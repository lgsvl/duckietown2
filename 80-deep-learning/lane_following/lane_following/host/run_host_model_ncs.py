import numpy as np
import cv2
import time
import socket
import struct
import time
from mvnc import mvncapi as mvnc
import os


devices = mvnc.EnumerateDevices()
device = mvnc.Device(devices[0])
device.OpenDevice()

BASE_PATH = os.path.dirname(os.path.realpath(__file__))
MODEL_NAME = 'caffe_model_2.graph'

with open('{}/model/caffe/{}'.format(BASE_PATH, MODEL_NAME), mode='rb') as f:
    graph_in_memory = f.read()

graph = device.AllocateGraph(graph_in_memory)


print "Loaded graph: {}".format(MODEL_NAME)
port = 12322
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("0.0.0.0", port))
sock.listen(1)
print "Start listening on port {}".format(port)

try:
	while True:
		conn, addr = sock.accept()
		try:
			imagelen = None
			buf = ""
			print "Connected from", addr
			while True:
				data = conn.recv(4 * 1024)
				if not data:
					print "Disconnected"
					break
				buf += data
				if imagelen is None:
					if len(buf) < 4:
						continue
					imagelen = struct.unpack_from("<I", buf)[0]
					buf = buf[4:]
				if len(buf) < imagelen:
					continue
				data = buf[0:imagelen]
				buf = buf[imagelen:]
				imagelen = None
				image = cv2.imdecode(np.frombuffer(data, np.uint8), -1)
				image = cv2.normalize(image.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
				# image = np.moveaxis(image, 2, 0)
				# image = np.expand_dims(image, axis=0)
				
				t1 = time.time()
				graph.LoadTensor(image.astype(np.float16), 'user object')
				preds, _ = graph.GetResult()
				output = preds[0]
				t2 = time.time()

				print "Time = %.3f omega = %.3f" % ((t2-t1), output)
				conn.send(struct.pack("<f", output))
		finally:
			conn.close()
finally:
	sock.close()
