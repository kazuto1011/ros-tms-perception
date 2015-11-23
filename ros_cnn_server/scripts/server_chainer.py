import numpy as np
import rospy as rp
import cv2
import time
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from config import config

import chainer
import chainer.functions as F
from chainer.functions import caffe

__author__ = 'kazuto1011'

TOPIC_NAME = "/camera/image/compressed"


class CNNClassifier:
    def __init__(self):
        rp.loginfo("Initialization")
        cv2.namedWindow("rgb", cv2.CV_WINDOW_AUTOSIZE)

        self.bridge = CvBridge()
        self.rgb_subscriber = rp.Subscriber(TOPIC_NAME, CompressedImage, self.classify, queue_size=1)

        # initialize a classifier
        self.caffe_net = caffe.CaffeFunction(config.path.caffe.caffemodel)

        # load synset_words
        self.categories = np.loadtxt(config.path.caffe.synset_words, str, delimiter="\t")

    def predict(self, x):
        y, = self.caffe_net(inputs={'data': x}, outputs=['fc8'], train=False)
        return F.softmax(y)

    def classify(self, image):
        # convert CompressedImage to numpy array
        np_arr = np.fromstring(image.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        im_size = 224
        rgb_image = cv2.resize(rgb_image, (im_size, im_size))

        batch = np.zeros((1, 3, im_size, im_size), dtype='float32')
        batch[0] = rgb_image.transpose(2, 0, 1)

        x = chainer.Variable(batch, volatile=True)

        # prediction
        start = time.time()
        p_dist = self.predict(x)
        index = np.argmax(p_dist.data)
        rp.loginfo("%.2f s. " % (time.time() - start) + self.categories[index])

        cv2.imshow("rgb", rgb_image)
        cv2.waitKey(30)


class NodeMain:
    def __init__(self):
        rp.init_node('ros_cnn_server', anonymous=False)
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
