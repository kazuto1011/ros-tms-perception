# tms_ss_rcnn
ROS-based module to detect multiple objects from an image. Core function is dependent on [Faster R-CNN (Python)](https://github.com/rbgirshick/py-faster-rcnn).

## Requirements
* [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn) installation

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
$ rosrun tms_ss_rcnn faster_rcnn_pascalvoc_gpu.py
```
The run the client node.
```sh
# Load an external image
$ rosrun tms_ss_rcnn clinet_loaded_image.py
# Live demo
$ rosrun tms_ss_rcnn clinet_uvc.py
```
If you want to publish images from your Android device, install [tms_ss_andtoid](../tms_ss_android) app to yours and run it with a identical master URI. Refer to this [link](https://github.com/irvs/ros_tms/wiki/how-to-configure-rosjava-apps-with-gradle) for installation of rosjava apps.
