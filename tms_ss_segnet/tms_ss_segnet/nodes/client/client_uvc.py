#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-07-07

import sys
import cv2
import numpy as np
import rospy as rp
import tms_ss_segnet.srv as srv
from sensor_msgs.msg import CompressedImage


class SegNetClient:

    def __init__(self):
        rp.wait_for_service('semantic_segmentation')

        self.cap = cv2.VideoCapture(0)

        try:
            self._client = rp.ServiceProxy(
                'semantic_segmentation', srv.segmentation)
        except rp.ServiceException, e:
            print 'Service call failed: %s' % e

        self.execute()

    def execute(self):
        while not rp.is_shutdown():
            ret, image = self.cap.read()
            req = self._convert2msg(image)
            res = self._client(req)
            self._visualize(image, res.result)

    @staticmethod
    def _convert2msg(img):
        req = CompressedImage()
        req.header.stamp = rp.Time.now()
        req.format = "jpeg"
        req.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        return req

    @staticmethod
    def _visualize(img, res):
        np_arr = np.fromstring(res.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        cv2.imshow('result', image_np)
        cv2.waitKey(10)


def main(args):
    rp.init_node('segnet_client', anonymous=True)
    SegNetClient()

    try:
        rp.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
