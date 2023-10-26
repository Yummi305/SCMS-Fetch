#ifndef FOLLOWTARGET.H
#define FOLLOWTARGET.H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3_vector.h"
#include "sensor_msgs/LaserScan.h"
#include "laserScan.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>


/*Class declerations here for followTarget.cpp*/
class FollowTarget{
    public:
        FollowTarget(ros::NodeHandle n);

        void tagCallback(const geometry_msgs::Vector3StampedPtr &msg); //reference point to input variable
        void laserCallback(const sensor_msgs::LaserScanPtr &smsg);
        void stop();

    private:
        ros::NodeHandle n_;
        ros::Subscriber laser;
        ros::Subscriber marker;
        ros::Subscriber posTracker;
        ros::Publisher vel;
        geometry_msgs::Twist teleop;

};

#endif