# SnCProject

This is the repository that our group is using for the Fetch robot to follow a path from a guider using a QR code. It uses Ubuntu 20.4 and ROS Noetic.

## Authors and contributions
* DENNIS N
  * Git setup
  * Gazebo world
  * Main launch
  * AcUro marker
  * Main.cpp threading
  * LaserProcessing programming
  * Obstacle Detection
  * FollowTarget logic and class setup
  * Bug fixes
* DANIEL M
  * Main launch
  * ArUco marker setup
  * followTarget launch
  * FollowTarget class and logic setup
  * CMakeList and package.xml edits
  * Bug fixes
* JENNIFER W
  * Teleop configuration
  * Spawn.launch
  * Bug fixes

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
git clone https://github.com/ROBOTIS-GIT/turtlebot3
cd ~/catkin_ws
catkin_make
```
```bash
cd ~/catkin_ws/src/fetch_gazebo/fetch_gazebo/launch
grep "xacro.py" -rli | xargs sed -i 's/xacro.py/xacro/g'
```

```bash
cd ~/catkin_ws/src
git clone https://github.com/PuerkitoBio/fetchbot
cd ~/catkin_ws
catkin_make
```

```bash
sudo apt install ros-noetic-aruco-ros ros-noetic-fetch-description ros-noetic-rgbd-launch ros-noetic-moveit-core ros-noetic-robot-controllers 
```


## Operation
Once the required packages and dependancies have been acquired, the following commands will launch the world, fetch robot and guider robot as well as the controller for the turtlebot.
* Open world with Fetch and Turtlebot, which also runs the command to control the turtlebot (using joystick controller) and RVIZ
```bash
roslaunch fetch_follow_target main.launch
```
* Fetch follows guider
```bash
roslaunch fetch_follow_target followTarget.launch
```




