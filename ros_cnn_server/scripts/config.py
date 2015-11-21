import sys
from easydict import EasyDict as edict

config = edict()

config.gpuNum = 0

config.path = edict()
config.path.caffe = edict()

config.path.caffe.rootDir = '/home/kazuto/caffe'
config.path.caffe.caffemodel = config.path.caffe.rootDir + '/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel'
config.path.caffe.prototxt = config.path.caffe.rootDir + '/models/bvlc_reference_caffenet/deploy.prototxt'
config.path.caffe.synset_words = config.path.caffe.rootDir + '/data/ilsvrc12/synset_words.txt'
