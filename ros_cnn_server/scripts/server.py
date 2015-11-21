import numpy as np
import cv2
import rospy as rp
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import caffe
import time
from config import config

__author__ = 'kazuto1011'

TOPIC_NAME = "/camera/image/compressed"


class CNNClassifier:
    def __init__(self):
        rp.loginfo("Initialization")

        self.bridge = CvBridge()
        self.rgb_subscriber = rp.Subscriber(TOPIC_NAME, CompressedImage, self.classify, queue_size=1)

        cv2.namedWindow("rgb", cv2.CV_WINDOW_AUTOSIZE)

        # initialize a classifier
        caffe.set_device(config.gpuNum)
        caffe.set_mode_gpu()
        self.classifier = caffe.Classifier(config.path.caffe.prototxt, config.path.caffe.caffemodel)

        # load synset_words
        self.categories = np.loadtxt(config.path.caffe.synset_words, str, delimiter="\t")

    def classify(self, image):
        # convert CompressedImage to numpy array
        np_arr = np.fromstring(image.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # prediction
        start = time.time()
        predictions = self.classifier.predict([rgb_image], oversample=False)
        index = np.argmax(predictions)
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
