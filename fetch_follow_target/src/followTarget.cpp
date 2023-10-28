#include "followTarget.h"
#include "laserprocessing.h"

// FollowTarget Constructor
FollowTarget::FollowTarget(ros::NodeHandle nh) : nh(nh)
{
    // ROS Subscriber
    laser_subscribe_ = nh.subscribe("orange/laser/scan", 100, &FollowTarget::laserCallback, this);

    // ROS Service
    marker_sub = nh.subscribe("/aruco_single/position", 1000, &FollowTarget::markerCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    sweepComplete = false;
    objectDetected = false;
    fetchMission = true;
    ROS_INFO_STREAM("FollowTarget Fetch created");
}

// Default destructor when program ends.
FollowTarget::~FollowTarget()
{
    if (laserProcessingPtr_ != nullptr)
    {
        delete laserProcessingPtr_;
    }
}

void FollowTarget::run()
{
    ROS_INFO_STREAM("run function");
    // While loop so that robot is always looking for target.
    while (ros::ok())
    {
    ROS_INFO_STREAM("run function while loop");

        if (fetchMission == false)
        {
            ROS_INFO_STREAM("fetchMission is false.");
        }
        else
        {
            ROS_INFO_STREAM("fetchMission is true.");
        }

        // if (fetchMission == false)
        // {
        //     ROS_INFO_STREAM("Fetch not moving forward due to obstacle.");

        //     // Stop Fetch from driving forward and colliding with obstacle
        //     stop();
        //     continue;
        // }
        // else
        // {
        //     ROS_INFO_STREAM("Fetch still running.");
        // }

        ROS_INFO_STREAM("About to make laser.");
        // New Laser scan.
        LaserProcessing laser;

        ROS_INFO_STREAM("Hopefully a laser has been made before this.");

        ROS_INFO_STREAM("About to make a new laser scan.");

        // Process laser reading.
        laser.newScan(laser_scan_);

        ROS_INFO_STREAM("Hopepfully a new laser has been made.");

        ROS_INFO_STREAM("About to check for an obstacle.");
        // Check if obstacle is blocking robot.
        if (laser.checkObstacle())
        {
            ROS_INFO_STREAM("OBSTACLED DETECTED.");
            // fetchMission = false;
        } else {
            ROS_INFO_STREAM("NO OBSTACLE DETECTED.");
        }
    }
}

void FollowTarget::stop()
{
}

void FollowTarget::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_scan_ = *msg;
}

void FollowTarget::markerCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!markerDetected)
    {
        markerDetected = true;
        ROS_INFO_STREAM("Guider has been detected");
        sweepComplete = true;
    }

    pose.vector.x = msg->vector.z;
    pose.vector.y = msg->vector.x;
    pose.vector.z = msg->vector.y;

    shortDist = roundf64(sqrt(pow(pose.vector.x, 2) + pow(pose.vector.y, 2)) * 10) / 10;
    thresholdErr = 0.01;

    if (shortDist <= (target_distance_threshold + thresholdErr) && shortDist >= (target_distance_threshold - thresholdErr))
    {

        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    else
    {
        if (!objectDetected)
        {
            double error = shortDist - target_distance_threshold;
            if (shortDist > target_distance_threshold * 1.5)
            {
                cmd_vel.linear.x = error;
            }
            else if (shortDist < target_distance_threshold * 0.95)
            {
                cmd_vel.linear.x = -0.3;
            }
            else
                cmd_vel.linear.x = error / 2;

            if (pose.vector.y == 0)
            {
                cmd_vel.angular.z = 0;
            }
            else
                cmd_vel.angular.z = -pose.vector.y;
        }
    }

    cmd_vel_pub.publish(cmd_vel);
}
