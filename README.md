# SnCProject

This is the repository that our group is using for the Fetch robot to follow a path from a guider using a QR code. It uses Ubuntu 20.4 and ROS Noetic.

## Authors and contributions
* DENNIS N
  * contributions
* DANIEL M
  * contributions
* JENNIFER W
  * contributions


## Dependancies
The following are required to be installed within your catkin workspace.
* Fetch packages
* Turtlebot packages

## Installations
Please install the following packages:
```bash
cd ~/catkin_ws/src
git clone https://github.com/Yummi305/SCMS-Fetch
cd ~/catkin_ws
catkin_make
```
```bash
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/aruco_ros
cd ~/catkin_ws
catkin_make

## Operation
Once the required packages and dependancies have been acquired, the following commands will launch the world, fetch robot and guider robot.
* Open world with Fetch and Turtlebot
```bash
roslaunch fetch_follow_target main.launch
```
* Fetch follows guider
```bash
roslaunch fetch_follow_target followTarget.launch
```
runs code that allows fetch to follow marker


