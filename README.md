# tms-perception
![ros-tms](https://avatars2.githubusercontent.com/u/8273459?v=3&s=200)  
Perception packages for [irvs/ros_tms](https://github.com/irvs/ros_tms).  

## tms_ss_rcnn
ROS-based client/server nodes to detect multiple objects from an image
* Object detection server wrapping the [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn)
* Android app to stream camera images

**Requirements**
* [Faster R-CNN (Python)](https://github.com/rbgirshick/py-faster-rcnn) installation

**Messaging**  
```sh
# Request: An image compressed to jpeg format
sensor_msgs/CompressedImage image
---
# Response: Arrays contains a class, score and region
tms_ss_rcnn/object[] objects
```

**Demo**  
Install the app to any Android device and run the server.  
Please refer to this [link](https://github.com/irvs/ros_tms/wiki/how-to-configure-rosjava-apps-with-gradle) for installation of rosjava apps.
```sh
# Download a trained model
$ roscd tms_ss_rcnn
$ cd models
$ bash fetch_segmentation_models.sh
```
```sh
# Server
## For CPU
$ rosrun tms_ss_rcnn faster_rcnn.py --cpu
## For GPU
$ rosrun tms_ss_rcnn faster_rcnn_gpu.py
```

## tms_ss_segnet
A server for semantic segmentation

**Requirements**
* [SegNet](https://github.com/alexgkendall/SegNet-Tutorial) installation

**Messaging**
```sh
# Request
sensor_msgs/CompressedImage image
---
# Response
sensor_msgs/CompressedImage image
```

**Demo**  
```sh
# Client
$ rosrun tms_ss_segnet clinet_uvc.py
```
```sh
# Server
$ rosrun tms_ss_segnet segnet.py
```

## tms_ss_cnn
Simple image recognition nodes

**Demo**  
```sh
$ rosrun tms_ss_cnn server_caffe.py
```

```sh
$ rosrun tms_ss_cnn server_chainer.py
```
