# Realsense locate object ROS2
## Introduction
-Use realsense rgbd camera along with YOLOX-ROS to create transforms from camera to different detect objects.
-Centre points of bounding boxes are used for calculations with aligned corresponding depths are used along with
  https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates for the same.
-Object corresponding X,Y,Z values will be published through a separate topic after few tests.
## Installation
In a ROS2 Workspace run
```
git clone https://github.com/Ar-Ray-code/bbox_ex_msgs
git clone https://github.com/IntelRealSense/realsense-ros -b ros2-beta
git clone https://github.com/skpawar1305/rs_yolox
```
and build packages with colcon and source them.
Install python dependencies with
```
pip install pyrealsense2
```
Clone YOLOX with
```
git clone https://github.com/Megvii-BaseDetection/YOLOX
```
And include it to PYTHON Path. Check path using 'pwd' in terminal.
e.g.
```
export PYTHONPATH=${PYTHONPATH}:/home/skpawar1305/Desktop/YOLOX
```
## Launch
Launch with
```
ros2 launch rs_yolox detect_object.launch.py rviz:=true
```