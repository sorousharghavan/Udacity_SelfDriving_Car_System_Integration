This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team Information:

* Team Leader: Soroush Arghavan, soroush.arghavan@gmail.com
* Team Members:
  * Boqiang Hu, huboqiang@gmail.com
  * Sergei Dmitriev, sergeidmv@gmail.com
  * Ahmed Ghazal, ahmedghazal93@gmail.com
  
### Video of code being run with simulator
<a href="https://www.youtube.com/watch?v=Q63nTLkf6zg" target="_blank"><img src="http://img.youtube.com/vi/Q63nTLkf6zg/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="400" height="300" border="10" /></a>

### Project architecture

* DBW Node: Publishes throttle, brake and steering for the vehicle to use.
* Traffic Light Detection: Detect the traffic light and its color from the /image_color. Once identified, the traffic light and its position waypoint index is published.
* Waypoint Updater Node: A waypoint updater which subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints. Uses /traffic_waypoint to change the waypoint target velocities before publishing to /final_waypoints. 

### Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
chmod u+x ./src/twist_controller/dbw_node.py ./src/waypoint_loader/waypoint_loader.py ./src/waypoint_updater/waypoint_updater.py ./src/tl_detector/tl_detector.py ./src/tl_detector/light_publisher.py ./src/camera_info_publisher/yaml_to_camera_info_publisher.py ./src/styx/server.py ./src/styx/unity_simulator_launcher.sh

roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
