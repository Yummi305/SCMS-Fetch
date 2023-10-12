#ifndef FOLLOW_TARGET_H
#define FOLLOW_TARGET_H

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"
#include "laserScan.h"
#include <cmath>
#include <chrono>
#include <vector>

class FollowTarget
{
public:
    FollowTarget(ros::NodeHandle nh);
    ~FollowTarget();

    void markerCallback(const geometry_msgs::Vector3StampedPtr &msg);
    void laserCallBack(const sensor_msgs::LaserScanConstPtr &msg);
    void stop();

private:
    ros::NodeHandle nh_;

    ros::Subscriber marker_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist twistMsg_;
    ros::Subscriber pose_tracker_;

    LaserDetection laserDetection_;

    double laser_readings_;
    bool obstacle_detected_;
    bool obstacle_reported_;
    bool search_reported_;
    bool sweep_complete_;
    tf2_msgs::TFMessageConstPtr pose_fetch_;

    struct Marker
    {
        geometry_msgs::Vector3Stamped pose;
        double threshold_distance;
        const double head_2_base_offset = 0.1088;
        double shortest_dist;
        bool detected;
        bool reached;
    };
    Marker marker_;

    ros::Time start_time_;   //!< start time
    ros::Duration duration_; //!< duration since start time (seconds)
};

#endif // GUIDER_FOLLOW_H