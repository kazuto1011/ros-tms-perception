import sys
from easydict import EasyDict as edict

cfg = edict()

cfg.gpuNum = 0

cfg.path = edict()
cfg.path.caffe = edict()

cfg.path.caffe.im_size = 224
cfg.path.caffe.rootDir = '/home/kazuto/caffe'
cfg.path.caffe.caffemodel = cfg.path.caffe.rootDir + '/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel'
cfg.path.caffe.prototxt = cfg.path.caffe.rootDir + '/models/bvlc_reference_caffenet/deploy.prototxt'
cfg.path.caffe.synset_words = cfg.path.caffe.rootDir + '/data/ilsvrc12/synset_words.txt'
