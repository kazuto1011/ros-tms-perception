# tms_ss_segnet
ROS-based module to perform pixelwise semantic segmentation implemented in [caffe-segnet](https://github.com/alexgkendall/caffe-segnet).

## Requirements
* [SegNet](https://github.com/alexgkendall/SegNet-Tutorial) installation

## Messaging
```sh
# Request: an image compressed to jpeg
sensor_msgs/CompressedImage image
---
# Response: a pixelwise labeled image
sensor_msgs/CompressedImage image
```

## Demo
Run the server node.
```sh
$ rosrun tms_ss_segnet segnet.py
```
Option
* --net [ DRIVING | SUN | PASCAL ]

Then run the client node.
```sh
# Load an external image
$ rosrun tms_ss_segnet clinet_loaded_image.py
# Live demo
$ rosrun tms_ss_segnet clinet_uvc.py
```
