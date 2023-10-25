#ifndef FOLLOWTARGET_H
#define FOLLOWTARGET_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// #include "geometry_msgs/Vector3_vector.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include "../include/laserprocessing.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include "std_srvs/SetBool.h"
#include "visualization_msgs/MarkerArray.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>


/*Class declerations here for followTarget.cpp*/
class FollowTarget{
    public:
        FollowTarget(ros::NodeHandle nh);
        ~FollowTarget();
        void run();
        void stop();
        void laserCallback(const sensor_msgs::LaserScanPtr &msg);
        void markerCallback(const geometry_msgs::Vector3StampedPtr &msg)
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

    private:
        // Laser data
        LaserProcessing *laserProcessingPtr_;
        bool fetchMission;
        double target_distance_threshold;
        double thresholdErr;
        double shortDist;
        bool sweepComplete;
        bool objectDetected;
        bool markerDetected
};

#endif