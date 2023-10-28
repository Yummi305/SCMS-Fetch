#ifndef FOLLOWTARGET_H
#define FOLLOWTARGET_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "laserprocessing.h"
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>


/*Class declerations here for followTarget.cpp*/
class FollowTarget{
    public:
        FollowTarget(ros::NodeHandle nh);
        ~FollowTarget();
        void run();
        void stop();
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void markerCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
        // void tagCallback(const geometry_msgs::Vector3StampedPtr &msg); //reference point to input variable
        
    protected:
        // ROS
        ros::NodeHandle nh;
        ros::Subscriber laser_subscribe_;
        ros::Subscriber posTracker;
        ros::Subscriber marker_sub;
        ros::Publisher cmd_vel_pub;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Vector3Stamped pose;

        // Threading and conditions
        std::thread *thread_;

        // Laser and range
        sensor_msgs::LaserScan laser_scan_;

    private:
        // Laser data
        LaserProcessing *laserProcessingPtr_;
        bool fetchMission;
        double target_distance_threshold;
        double thresholdErr;
        double shortDist;
        bool sweepComplete;
        bool objectDetected;
        bool markerDetected;
};

#endif