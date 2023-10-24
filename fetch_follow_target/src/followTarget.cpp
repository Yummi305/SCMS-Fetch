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
#include <istream>
#include <chrono>
#include <vector>
#include <cmath>

#define LASER_LIMIT 0.35 //in meters

class LaserScanning{
    public:
        LaserScanning();
    private:
        bool ObjectDetection(const sensor_msgs::LaserScan::ConstPtr& msg); //callback from laser, check if object has been detected

};

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

bool LaserScanning::ObjectDetection(const sensor_msgs::ConstPtr& msg){
    float laserScan = laserScan->range_max;
    float laserMin = laserScan->angle_min;
    float laserMax = laserScan->angle_max;
    for (int i = laserMin; i<=laserMax; i++){
        if (laserScan->ranges(i) < laserReading){
            laserScan = laserScan->ranges.at(i);
        }
    }
    if (laserScan <= LASER_LIMIT){
        return true;
    }
    else return false;
    
}

FollowTarget::FollowTarget(){

    /*
        laser, check if fetch is within range of guider
        if detection range is > 1m move towards guider.
    */

   /*
        Check for marker, if marker is small, then move. call laser scan.
        rotate fetch based on marker position transform
   */

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