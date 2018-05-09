import keras
from keras.models import load_model
from keras import backend as K
import numpy as np
import cv2
import time
import socket
import struct
import time
import tensorflow as tf
import os


config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
K.set_session(sess)

BASE_PATH = os.path.dirname(os.path.realpath(__file__))
MODEL_NAME = 'model_v12.h5'

model = load_model('{}/model/keras/{}'.format(BASE_PATH, MODEL_NAME))
# saver = tf.train.import_meta_graph('./model/tf/dl_model_inference.meta')
# saver.restore(sess, tf.train.latest_checkpoint('./model/tf/'))

print "Loaded model: {}".format(MODEL_NAME)
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
				image = np.expand_dims(image, axis=0)
				
				t1 = time.time()
				output = model.predict(image)
				# output = sess.run('output:0', feed_dict={'input:0': image})
				t2 = time.time()

				print "Time = %.3f omega = %.3f" % ((t2-t1), output)
				conn.send(struct.pack("<f", output))
		finally:
			conn.close()
finally:
	sock.close()
