## qr_detector
The QR codes detector based on zbar library (http://zbar.sourceforge.net), dedicated to ROS systems.

Installation of zbar library on Ubuntu
`sudo apt install libzbar-dev`

Subscribes:
- **/image** (sensor_msgs/Image) - the topic with RGB images which contains QR codes.

Publishes:
- **/qr_codes** (std_msgs/String) - message from each detected QR code is published as a string.

### Installation
Git clone this package in to your workspace.
```
cd your_ws/src
git clone https://github.com/MinSungjae/qr_detector
catkin_make
```
Before building package, you have to make sure installing libzbar-dev.

### How to run the QR Detector Node
First, you have to run nodelet master before launching QR Detector node.
```
rosrun nodelet nodelet manager __name:=nodelet_manager
```

In other terminal, use nodelet load to launch QR Detector node.
```
rosrun nodelet nodelet load qr_detector/qr_detector_nodelet nodelet_manager
```
