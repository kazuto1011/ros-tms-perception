#!/usr/bin/env python
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy as rp
import argparse
import tms_ss_rcnn.srv
from sensor_msgs.msg import CompressedImage
from primesense import openni2

class RCNNClient:
    def __init__(self):
        rp.wait_for_service('faster_rcnn')

        openni2.initialize()

        self.dev = openni2.Device.open_any()
        print self.dev.get_sensor_info(openni2.SENSOR_COLOR)

        self.color_stream = self.dev.create_color_stream()

        try:
            self._client = rp.ServiceProxy('faster_rcnn', tms_ss_rcnn.srv.obj_detection)
        except rp.ServiceException, e:
            print 'Service call failed: %s' % e

        self.color_stream.start()
        self.execute()

    def execute(self):
        while not rp.is_shutdown():
            color_frame = self.color_stream.read_frame()
            color_data  = color_frame.get_buffer_as_uint8()
            color_array = np.ndarray((color_frame.height, color_frame.width, 3), dtype=np.uint8, buffer=color_data)
            color_array = cv2.flip(color_array, 1)
            req = self._convert2msg(color_array[:,:,::-1])
            res = self._client(req)
            self._visualize(color_array, res)

    @staticmethod
    def _convert2msg(img):
        req = CompressedImage()
        req.header.stamp = rp.Time.now()
        req.format       = "jpeg"
        req.data         = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        return req

    def _visualize(self, img, res):
        for obj in res.objects:
            tl_x = obj.region.x_offset
            tl_y = obj.region.y_offset
            br_x = tl_x + obj.region.width
            br_y = tl_y + obj.region.height
            cv2.rectangle(img, (tl_x, tl_y), (br_x, br_y), (0, 0, 255), 2)
            cv2.putText(img, obj.class_name, (tl_x, tl_y-2), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0 ,255), 2)
        
        cv2.imshow("color", img);
        cv2.waitKey(30)

def main(args):
    rp.init_node('faster_rcnn_client_xtion', anonymous=True)
    RCNNClient()

    try:
        rp.spin()
    except KeyboardInterrupt:
        openni2.unload()
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
