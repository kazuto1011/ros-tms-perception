#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-07-07

import numpy as np
import os.path as osp
import argparse
import cv2
import sys

import rospy as rp
from config import config
import tms_ss_segnet.srv as srv
from sensor_msgs.msg import CompressedImage

sys.path.append('/usr/local/lib/python2.7/site-packages')
caffe_root = config['caffe_root']
print caffe_root
sys.path.insert(0, osp.join(caffe_root, 'python'))
import caffe

models_dir = osp.join(config['segnet_root'], 'Example_Models')
scripts_dir = osp.join(config['segnet_root'], 'Scripts')


NETS = {
    'DRIVING': (
        'segnet_model_driving_webdemo.prototxt',
        'segnet_weights_driving_webdemo.caffemodel'
    ),
    'SUN': (
        'segnet_sun.prototxt',
        'segnet_sun.caffemodel'
    ),
    'PASCAL': (
        'segnet_pascal.prototxt',
        'segnet_pascal.caffemodel'
    )
}


def parse_args():
    parser = argparse.ArgumentParser(description='Semantic Segmentation')
    parser.add_argument('--net', type=str, default='SUN')
    parser.add_argument('--colours', type=str, default='sun.png')
    args = parser.parse_args()
    return parser.parse_args()


class SegNet:

    def __init__(self, name, args, net):
        self.args = args
        self.name = name
        self.init = False
        self.net = net

        rp.loginfo("Ready to start")
        self.server = rp.Service(self.name, srv.segmentation, self._callback)

    def _callback(self, req):
        rp.loginfo('Received a request')

        if self.init is False:
            self.net = caffe.Net(
                osp.join(models_dir, NETS[self.net][0]),
                osp.join(models_dir, NETS[self.net][1]),
                caffe.TEST
            )
            caffe.set_mode_gpu()

            rp.loginfo('Loaded a model')
            self.init = True
            self.input_shape = self.net.blobs['data'].data.shape
            self.output_shape = self.net.blobs['argmax'].data.shape
            self.label_colours = cv2.imread(
                osp.join(scripts_dir, self.args.colours)).astype(np.uint8)

        image = self._msg2cv(req.image)

        image = cv2.resize(image, (self.input_shape[3], self.input_shape[2]))
        input_image = image.transpose((2, 0, 1))
        input_image = np.asarray([input_image])
        out = self.net.forward_all(data=input_image)

        segmentation_ind = np.squeeze(self.net.blobs['argmax'].data)
        segmentation_ind_3ch = np.resize(
            segmentation_ind, (3, self.input_shape[2], self.input_shape[3]))
        segmentation_ind_3ch = segmentation_ind_3ch.transpose(
            1, 2, 0).astype(np.uint8)
        segmentation_rgb = np.zeros(
            segmentation_ind_3ch.shape, dtype=np.uint8)

        cv2.LUT(segmentation_ind_3ch, self.label_colours, segmentation_rgb)

        return self._cv2msg(segmentation_rgb)

    @staticmethod
    def _cv2msg(img):
        res = CompressedImage()
        res.header.stamp = rp.Time.now()
        res.format = "jpeg"
        res.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        return res

    @staticmethod
    def _msg2cv(msg):
        np_array = np.fromstring(msg.data, np.uint8)
        return cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)

class NodeMain:

    def __init__(self):
        rp.init_node('segnet_server', anonymous=False)
        rp.on_shutdown(self.shutdown)

        args = parse_args()
        node = SegNet('semantic_segmentation', args, args.net)

        rp.spin()

    @staticmethod
    def shutdown():
        cv2.destroyAllWindows()
        rp.loginfo("Shutting down")


if __name__ == '__main__':
    try:
        NodeMain()
    except rp.ROSInterruptException:
        rp.loginfo("Terminated")
