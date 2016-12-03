# tms_ss_ssd
ROS-based module to detect multiple objects from an image. Core function is dependent on [Single Shot Detector (SSD)](https://github.com/weiliu89/caffe).

## Requirements
* [weiliu89/caffe](https://github.com/weiliu89/caffe) installation
  * checkout ```ssd``` branch

## Messaging
```sh
# Request: an image compressed to jpeg
sensor_msgs/CompressedImage image
---
# Response: message arrays contains a class, score and region
tms_ss_rcnn/object[] objects
```

## Demo
Run the server node.
```sh
# PASCAL VOC
$ rosrun tms_ss_ssd ssd_pascalvoc.py
# MS COCO
$ rosrun tms_ss_ssd ssd_coco.py
```
The run the client node.
```sh
# Load an external image
$ rosrun tms_ss_rcnn clinet_loaded_image.py
# Live demo
$ rosrun tms_ss_rcnn clinet_uvc.py
```
