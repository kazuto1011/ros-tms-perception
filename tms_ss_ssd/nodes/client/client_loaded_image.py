#!/usr/bin/env python
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy as rp
import argparse
import tms_ss_ssd.srv
from sensor_msgs.msg import CompressedImage

def parse_args():
    parser = argparse.ArgumentParser(description='Single-Shot Multibox Detector demo client')
    parser.add_argument('--image', dest='image',
                        help='Image to detect',
                        default='/home/kazuto/Desktop/test.png')
    return parser.parse_args()

class SSDClient:
    def __init__(self):
        rp.wait_for_service('ssd')
        try:
            self._client = rp.ServiceProxy('ssd', tms_ss_ssd.srv.obj_detection)

            img = cv2.imread(parse_args().image, cv2.IMREAD_COLOR)
            req = self._convert2msg(img)
            res = self._client(req)
            self._visualize(img, res)

        except rp.ServiceException, e:
            print 'Service call failed: %s' % e

    @staticmethod
    def _convert2msg(img):
        req = CompressedImage()
        req.header.stamp = rp.Time.now()
        req.format       = "jpeg"
        req.data         = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        return req

    @staticmethod
    def _visualize(img, res):
        img = img[:, :, (2, 1, 0)]
        fig, ax = plt.subplots(figsize=(12, 12))
        ax.imshow(img, aspect='equal')
        for obj in res.objects:
            ax.add_patch(
                    plt.Rectangle((obj.region.x_offset, obj.region.y_offset),
                                  obj.region.width,
                                  obj.region.height,
                                  fill=False, edgecolor='red', linewidth=3.5)
            )
            ax.text(obj.region.x_offset, obj.region.y_offset - 2,
                    '{:s} {:.3f}'.format(obj.class_name, obj.score),
                    bbox=dict(facecolor='blue', alpha=0.5),
                    fontsize=14, color='white')

        plt.axis('off')
        plt.tight_layout()
        plt.draw()
        plt.show()


def main(args):
    rp.init_node('ssd_client', anonymous=True)
    ic = SSDClient()

    try:
        rp.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
