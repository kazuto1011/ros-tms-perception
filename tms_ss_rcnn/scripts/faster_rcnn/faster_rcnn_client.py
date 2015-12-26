#!/usr/bin/env python
import sys, time, cv2
import numpy as np
from scipy.ndimage import filters
import rospy as rp

import tms_ss_rcnn.srv
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import RegionOfInterest

class image_feature:

    def __init__(self):
        rp.wait_for_service('faster_rcnn')
        try:
            self._client = rp.ServiceProxy('faster_rcnn', tms_ss_rcnn.srv.obj_detection)

            image_np = cv2.imread('/home/kazuto/Desktop/cat.jpg', cv2.IMREAD_COLOR)

            msg = CompressedImage()
            msg.header.stamp = rp.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

            res = self._client(msg)
            print res.height

        except rp.ServiceException, e:
            print 'Service call failed: %s'%e

def main(args):
    rp.init_node('faster_rcnn_client', anonymous=False)
    ic = image_feature()
    try:
        rp.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
