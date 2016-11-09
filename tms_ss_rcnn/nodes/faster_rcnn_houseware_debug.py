#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-07-05

import os
import os.path as osp
import sys

if 'FRCN_ROOT' not in os.environ:
    print "Could not find $FRCN_ROOT."
    sys.exit(1)

FRCN_ROOT = os.environ['FRCN_ROOT']
print "FRCNN=" + FRCN_ROOT
sys.path.insert(0, osp.join(FRCN_ROOT, 'caffe-fast-rcnn/python'))
sys.path.insert(0, osp.join(FRCN_ROOT, 'lib'))

from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import numpy as np
import caffe
import cv2
import argparse
import rospy as rp
import tms_ss_rcnn.srv as srv
import tms_ss_rcnn.msg as msg

CLASSES = ('__background__',
           'Water bottle',
           'Wine bottle',
           'Book',
           'Teapot',
           'Watering pot',
           'Kettle',
           'Coffee can',
           'Glass',
           'Cup',
           'Dish',
           'Controller'
           )

NETS = {'vgg16': ('VGG16', 'vgg16_housewares.caffemodel'),
        'zf': ('ZF', 'zf_housewares.caffemodel')}

def parse_args():
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id',
                        help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
    parser.add_argument('--net', dest='demo_net',
                        help='Network to use [vgg16]',
                        choices=NETS.keys(), default='zf')
    parser.add_argument('--conf', dest='conf_thresh',
                        default=0.8, type=float)
    parser.add_argument('--nms', dest='nms_thresh',
                        default=0.3, type=float)
    return parser.parse_args()


def load_net(args):
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    data_dir = osp.join(osp.abspath(osp.dirname(__file__)), '../models')
    models_dir = osp.join(data_dir, 'houseware')
    prototxt = osp.join(models_dir, NETS[args.demo_net][
                        0], 'faster_rcnn_end2end', 'test.prototxt')
    caffemodel = osp.join(data_dir, NETS[args.demo_net][1])

    if args.cpu_mode:
        caffe.set_mode_cpu()
    else:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        cfg.GPU_ID = args.gpu_id

    return caffe.Net(prototxt, caffemodel, caffe.TEST)


class FasterRCNN:

    def __init__(self, name, args, conf_thresh, nms_thresh):
        self._args = args
        self._name = name
        self._init = False
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh

        rp.loginfo("Ready to start")
        self._server = rp.Service(
            self._name, srv.obj_detection, self._callback)

    def _callback(self, req):
        rp.loginfo("Received an image")

        if self._init is False:
            self._net = load_net(self._args)
            self._init = True

        if self._args.cpu_mode:
            caffe.set_mode_cpu()
        else:
            caffe.set_mode_gpu()
            caffe.set_device(self._args.gpu_id)
            cfg.GPU_ID = self._args.gpu_id

        # convert rosmsg to cv image
        np_array = np.fromstring(req.image.data, np.uint8)
        image = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)

        objects = self._detect(image)
        return srv.obj_detectionResponse(objects)

    def _detect(self, image):
        # Detect all object classes and regress object bounds
        timer = Timer()
        timer.tic()
        scores, boxes = im_detect(self._net, image)
        timer.toc()

        print ('Detection took {:.3f}s for '
               '{:d} object proposals').format(timer.total_time, boxes.shape[0])

        return self._post_process(scores, boxes, image)

    def _post_process(self, scores, boxes, image):
        obj_list = []
        for cls_ind, cls in enumerate(CLASSES[1:]):
            cls_ind += 1  # because we skipped background
            cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack(
                (cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, self.nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= self.conf_thresh)[0]
            if len(inds) == 0:
                continue

            for i in inds:
                obj = msg.object()
                obj.class_name = cls
                obj.score = dets[i, -1]

                bbox = dets[i, :4]
                obj.region.x_offset = bbox[0]
                obj.region.y_offset = bbox[1]
                obj.region.width = bbox[2] - bbox[0]
                obj.region.height = bbox[3] - bbox[1]
                obj.region.do_rectify = False

                #-Added---------------------------------------------------------
                cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
                cv2.putText(image, obj.class_name, (int(bbox[0]), int(bbox[1]-2)), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 2)
                #---------------------------------------------------------------

                obj_list.append(obj)

        #-Added-----------------------------------------------------------------
        cv2.imshow("server", image)
        cv2.waitKey(10)
        #-----------------------------------------------------------------------

        return obj_list


class NodeMain:

    def __init__(self):
        rp.init_node('faster_rcnn', anonymous=False)
        rp.on_shutdown(self.shutdown)

        args = parse_args()
        node = FasterRCNN('faster_rcnn', args, conf_thresh=args.conf_thresh, nms_thresh=args.nms_thresh)

        rp.spin()

    @staticmethod
    def shutdown():
        rp.loginfo("Shutting down")


if __name__ == '__main__':
    try:
        NodeMain()
    except rp.ROSInterruptException:
        rp.loginfo("Terminated")
