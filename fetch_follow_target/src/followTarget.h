#ifndef FOLLOWTARGET_H
#define FOLLOWTARGET_H

#include "laserprocessing.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include "std_srvs/SetBool.h"
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
        void laserCallback(const sensor_msgs::LaserScanPtr &msg);
        void tagCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
        void stop();
        void followAruco(const geometry_msgs::Vector3 &msg);
        double DistancetoMarker(const geometry_msgs::Vector3 &msg);
        
    protected:
        // ROS
        ros::NodeHandle nh;
        ros::Subscriber laser_subscribe_;
        ros::Subscriber marker_sub;
        ros::Publisher cmd_vel_pub;
        tf::TransformListener* listener;
        tf::StampedTransform transform;
        geometry_msgs::Twist cmd_vel;

        // Laser and range
        sensor_msgs::LaserScan laser_scan_;

    private:
        // Laser data
        LaserProcessing *laserProcessingPtr_;
        bool sweepComplete;
        bool objectDetected;
        bool objectReported;
        bool searchReported;

        struct Tag{
            geometry_msgs::Vector3Stamped pose;
            double distanceLimit;
            double shortDist;
            double thresholdErr;
            bool detected;
            bool reached;
            double marker_x,marker_y,marker_z;
            double robot_x,robot_y,robot_z;
        };

        Tag ARUCO;

        ros::Time start;
        ros::Duration duration;
};

#endif