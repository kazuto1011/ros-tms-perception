# rcnn-server
ROS-based client/server nodes to detect multiple objects from an image
* A server using Convolutional Neural Networks for object detection
* Android app to stream camera images

## Prerequisite
* [ROS (our lab page)](https://github.com/irvs/ros_tms/wiki/install)
* [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn)

## Demo
***UNDER DEVELOPMENT***
### Android app
You need to open the project on [Android Studio](http://developer.android.com/sdk/index.html), and install the app to device.  
Please refer to this. [link](https://github.com/irvs/ros_tms/wiki/how-to-configure-rosjava-apps-with-gradle)

### Detection server
```sh
$ rosrun tms_ss_rcnn faster_rcnn.py --cpu
```
or
```sh
$ rosrun tms_ss_rcnn faster_rcnn_gpu_workaround.py
```
