# cnn-server
* ROS nodes using convolutional neural network for image classification
* Android app to stream camera images

## Prerequisite
* [Android Studio](http://developer.android.com/sdk/index.html)
* [ROS (our lab page)](https://github.com/irvs/ros_tms/wiki/install)
* [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn)*
* [Caffe](https://github.com/BVLC/caffe)**
* [Chainer](https://github.com/pfnet/chainer)**

## Usage
***UNDER DEVELOPMENT. Actually, the followings cannot work together for now.***
### Android app
You need to open the project on Android Studio, and install the app to device.  
Please refer to this. [link](https://github.com/irvs/ros_tms/wiki/how-to-configure-rosjava-apps-with-gradle)

### CNN server
Object detection*
```sh
$ rosrun tms_ss_rcnn faster_rcnn.py [--cpu]
```
Image classification**
```sh
$ rosrun tms_ss_rcnn server_caffe.py
```
```sh
$ rosrun tms_ss_rcnn server_chainer.py
```
