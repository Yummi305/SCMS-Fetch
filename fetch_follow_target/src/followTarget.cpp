#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <iostream>

class FollowTarget {

    ros::NodeHandle nh;
    ros::Subscriber marker_sub;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Vector3Stamped pose;

    double target_distance_threshold;
    double thresholdErr;
    double shortDist;
    bool sweepComplete;
    bool objectDetected;
    bool markerDetected;

    public:
    FollowTarget(ros::NodeHandle nh) {
        marker_sub = nh.subscribe("/aruco_single/position", 1000, &FollowTarget::markerCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
        
        target_distance_threshold = 1.0;
        ROS_INFO_STREAM("init");

        sweepComplete = false;
        objectDetected = false;
    }

    void markerCallback(const geometry_msgs::Vector3StampedPtr& msg){
        if(!markerDetected){
            markerDetected = true;
            ROS_INFO_STREAM("Guider has been detected");
            sweepComplete = true;
        }
        
        pose.vector.x = msg->vector.z;
        pose.vector.y = msg->vector.x;
        pose.vector.z = msg->vector.y;

        shortDist = roundf64(sqrt(pow(pose.vector.x,2) +pow(pose.vector.y,2))*10)/ 10;
        thresholdErr = 0.01;

        if (shortDist <= (target_distance_threshold + thresholdErr) && shortDist >= (target_distance_threshold - thresholdErr)){

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

        }

        else {
            if(!objectDetected){
                double error = shortDist - target_distance_threshold;
                if (shortDist > target_distance_threshold *1.5){
                    cmd_vel.linear.x = error;
                }
                else if (shortDist < target_distance_threshold *0.95){
                    cmd_vel.linear.x = -0.3;
                }
                else cmd_vel.linear.x = error/2;

                if (pose.vector.y ==0){
                    cmd_vel.angular.z =0;
                }
                else cmd_vel.angular.z =- pose.vector.y;
            }
        }

        cmd_vel_pub.publish(cmd_vel);
    }

};

int main(int argc, char** argv){
    ros::init(argc,argv,"followTarget");
    FollowTarget followTarget;
    ros::spin();
    return 0;
}