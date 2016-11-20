#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

if 'SSD_ROOT' not in os.environ:
    print "Could not find 'SSD_ROOT'."
    sys.exit(1)

SSD_ROOT = os.environ['SSD_ROOT']
print "SDD=" + SSD_ROOT
sys.path.insert(0, os.path.join(SSD_ROOT, 'python'))

import caffe
import cv2
import argparse
import rospy as rp
import tms_ss_ssd.srv as srv
import tms_ss_ssd.msg as msg

from google.protobuf import text_format
from caffe.proto import caffe_pb2

# load PASCAL VOC labels
labelmap_file = os.path.join(SSD_ROOT, 'data/VOC0712/labelmap_voc.prototxt')
file = open(labelmap_file, 'r')
labelmap = caffe_pb2.LabelMap()
text_format.Merge(str(file.read()), labelmap)

def get_labelname(labelmap, labels):
    num_labels = len(labelmap.item)
    labelnames = []
    if type(labels) is not list:
        labels = [labels]
    for label in labels:
        found = False
        for i in xrange(0, num_labels):
            if label == labelmap.item[i].label:
                found = True
                labelnames.append(labelmap.item[i].display_name)
                break
        assert found == True
    return labelnames

def parse_args():
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id',
                        help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
    # parser.add_argument('--net', dest='demo_net',
    #                     help='Network to use [vgg16]',
    #                     choices=NETS.keys(), default='vgg16')
    parser.add_argument('--conf', dest='conf_thresh',
                        default=0.6, type=float)
    parser.add_argument('--nms', dest='nms_thresh',
                        default=0.3, type=float)
    return parser.parse_args()


def load_net(args):
    model_def = os.path.join(SSD_ROOT, 'models/VGGNet/VOC0712/SSD_300x300/deploy.prototxt')
    model_weights = os.path.join(SSD_ROOT, 'models/VGGNet/VOC0712/SSD_300x300/VGG_VOC0712_SSD_300x300_iter_60000.caffemodel')

    return caffe.Net(model_def, model_weights, caffe.TEST)


class SSD:
    def __init__(self, name, args):
        self._args = args
        self._name = name
        self._init = False

        rp.loginfo("Ready to start")
        self._server = rp.Service(self._name, srv.obj_detection, self._callback)

    def _callback(self, req):
        rp.loginfo("Received an image")

        if self._init is False:
            self._net = load_net(self._args)
            self.transformer = caffe.io.Transformer({'data': self._net.blobs['data'].data.shape})
            self.transformer.set_transpose('data', (2, 0, 1))
            self.transformer.set_mean('data', np.array([104,117,123]))
            self.transformer.set_raw_scale('data', 255)
            self.transformer.set_channel_swap('data', (2,1,0))
            self._init = True

        if self._args.cpu_mode:
            caffe.set_mode_cpu()
        else:
            caffe.set_mode_gpu()
            caffe.set_device(self._args.gpu_id)

        # convert rosmsg to cv image
        np_array = np.fromstring(req.image.data, np.uint8)
        image = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)

        objects = self._detect(image)
        return srv.obj_detectionResponse(objects)

    def _detect(self, image):
        # set net to batch size of 1
        image_resize = 300
        self._net.blobs['data'].reshape(1, 3, image_resize, image_resize)

        image = np.asarray(image, np.float32)
        image /= 255

        transformed_image = self.transformer.preprocess('data', image)
        self._net.blobs['data'].data[...] = transformed_image

        # Forward pass.
        detections = self._net.forward()['detection_out']

        # Parse the outputs.
        det_label = detections[0,0,:,1]
        det_conf = detections[0,0,:,2]
        det_xmin = detections[0,0,:,3]
        det_ymin = detections[0,0,:,4]
        det_xmax = detections[0,0,:,5]
        det_ymax = detections[0,0,:,6]

        # Get detections with confidence higher than 0.6.
        top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.6]

        top_conf = det_conf[top_indices]
        top_label_indices = det_label[top_indices].tolist()
        top_labels = get_labelname(labelmap, top_label_indices)
        top_xmin = det_xmin[top_indices]
        top_ymin = det_ymin[top_indices]
        top_xmax = det_xmax[top_indices]
        top_ymax = det_ymax[top_indices]

        colors = plt.cm.hsv(np.linspace(0, 1, 21)).tolist()

        currentAxis = plt.gca()

        obj_list = []

        for i in xrange(top_conf.shape[0]):
            xmin = int(round(top_xmin[i] * image.shape[1]))
            ymin = int(round(top_ymin[i] * image.shape[0]))
            xmax = int(round(top_xmax[i] * image.shape[1]))
            ymax = int(round(top_ymax[i] * image.shape[0]))
            score = top_conf[i]
            label = int(top_label_indices[i])
            label_name = top_labels[i]
            display_txt = '%s: %.2f'%(label_name, score)
            coords = (xmin, ymin), xmax-xmin+1, ymax-ymin+1
            color = colors[label]

            currentAxis.add_patch(plt.Rectangle(*coords, fill=False, edgecolor=color, linewidth=2))
            currentAxis.text(xmin, ymin, display_txt, bbox={'facecolor':color, 'alpha':0.5})

            obj = msg.object()
            obj.class_name = label_name
            obj.score = top_conf[i]

            obj.region.x_offset = xmin
            obj.region.y_offset = ymin
            obj.region.width    = xmax-xmin+1
            obj.region.height   = ymax-ymin+1
            obj.region.do_rectify = False

            obj_list.append(obj)

        return obj_list


class NodeMain:
    def __init__(self):
        rp.init_node('ssd', anonymous=False)
        rp.on_shutdown(self.shutdown)

        args = parse_args()
        node = SSD('ssd', args)

        rp.spin()

    @staticmethod
    def shutdown():
        rp.loginfo("Shutting down")


if __name__ == '__main__':
    try:
        NodeMain()
    except rp.ROSInterruptException:
        rp.loginfo("Terminated")
