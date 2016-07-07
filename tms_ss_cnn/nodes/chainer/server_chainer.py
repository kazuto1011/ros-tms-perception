#!/usr/bin/env python
import numpy as np
import time

import rospy as rp
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import chainer
import chainer.functions as F
from chainer.functions.caffe import CaffeFunction

from _init_paths import cfg

__author__ = 'kazuto1011'

SUB_TOPIC_NAME = "/camera/image/compressed"
PUB_TOPIC_NAME = ""


class CNNClassifier:
    def __init__(self):
        rp.loginfo("Initialization")
        cv2.namedWindow("rgb", cv2.CV_WINDOW_AUTOSIZE)

        # initialize a classifier
        start = time.time()
        self.caffe_net = CaffeFunction(cfg.path.caffe.caffemodel)
        rp.loginfo("%.2f s. Loaded a model" % (time.time() - start))

        # load synset_words
        self.categories = np.loadtxt(cfg.path.caffe.synset_words, str, delimiter="\t")

        self.im_size = cfg.path.caffe.im_size
        self.im_shape = (self.im_size, self.im_size)

        self.bridge = CvBridge()
        self.rgb_subscriber = rp.Subscriber(SUB_TOPIC_NAME, CompressedImage, self.classify, queue_size=1)

    def predict(self, x):
        y, = self.caffe_net(inputs={'data': x}, outputs=['fc8'], train=False)
        return F.softmax(y)

    def classify(self, image):
        # convert CompressedImage to numpy array
        np_arr = np.fromstring(image.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        rgb_image = cv2.resize(rgb_image, self.im_shape)

        batch = np.zeros((1, 3, self.im_size, self.im_size), dtype='float32')
        batch[0] = rgb_image.transpose(2, 0, 1)

        # prediction
        start = time.time()
        cls_score = self.predict(chainer.Variable(batch, volatile=True))
        index = np.argmax(cls_score.data)
        rp.loginfo("%.2f s. " % (time.time() - start) + self.categories[index])

        cv2.imshow("rgb", rgb_image)
        cv2.waitKey(30)


class NodeMain:
    def __init__(self):
        rp.init_node('server_chainer', anonymous=False)
        rp.on_shutdown(self.shutdown)

        CNNClassifier()

        rp.spin()
        cv2.destroyAllWindows()

    @staticmethod
    def shutdown():
        rp.loginfo("Shutting down")


if __name__ == '__main__':
    try:
        NodeMain()
    except rp.ROSInterruptException:
        rp.loginfo("Terminated")
