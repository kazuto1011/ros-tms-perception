#!/usr/bin/env python
# coding: utf-8
#
# Author:   Kazuto Nakashima
# URL:      https://github.com/kazuto1011
# Created:  2016-07-07

import os.path as osp
home = osp.expanduser("~")

config = {
    'segnet_root': osp.join(home, 'SegNet'),
    'caffe_root': osp.join(home, 'SegNet/caffe-segnet')
}
