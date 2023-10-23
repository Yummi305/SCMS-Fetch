/*
Goals:
- Setup up class which will house components for tracking the April Tag
- Class will interperate position of marker relative to fetch camera position
- Publish velocity commands to cmd_vel. 
- Setup object detection via laser scanning class

*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf_msgs/tfMessage.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/laserprocessing.h"
#include <istream>
#include <chrono>
#include <vector>
#include <cmath>



#define LASER_LIMIT 0.35 //in meters

class FollowTarget{
    public:
        FollowTarget();
        void stop();
        void markerClbk(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    
    private:
        LaserScanning laserScan;
        ros::NodeHandler n;
        ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1)

};

FollowTarget::FollowTarget(ros::NodeHandle n_){

    laser_subscribe_ = n_.subscribe("orange/laser/scan", 100, &FollowTarget::laserCallback, this);

    // While loop so that robot is always looking for target.
    while (ros::ok())
    {

        if (!fetchDrive)
        {
        ROS_INFO_STREAM("Fetch not moving forward due to obstacle.");
        // Stop Fetch from driving forward and colliding with obstacle
        stop();
        continue;
        } else {
            // Fetch may continue driving

        }

        // New Laser scan.
        LaserProcessing laser;

        // Process laser reading.
        laser.newScan(laser_scan_);

        // Check if obstacle is blocking robot.
        if (laser.checkObstacle())
        {
            ROS_INFO_STREAM("Obstacle detected in path.");
            fetchDrive = false;
        }

    }
}

FollowTarget::stop(){

}

FollowTarget::markerClbk(const geometry_msgs::Vector3Stamped::ConstPtr& msg){

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "followTarget");
  FollowTarget followTarget;
  ros::spin();
}



void FollowTarget::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  laser_scan_ = *msg;
}

