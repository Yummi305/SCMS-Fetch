#include "followTarget.h"
#include "laserprocessing.h"

// FollowTarget Constructor
FollowTarget::FollowTarget(ros::NodeHandle nh) : nh(nh)
{
    // ROS Subscriber
    laser_subscribe_ = nh.subscribe("/base_scan_raw", 100, &FollowTarget::laserCallback, this);

    // ROS Service
    marker_sub = nh.subscribe("/aruco_single/position", 1000, &FollowTarget::tagCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ARUCO.distanceLimit = 1.0;

    duration = start - start;

    sweepComplete = false;
    objectDetected = false;
    searchReported = false;

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

void FollowTarget::tagCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    ROS_INFO_STREAM("test");

    if (!ARUCO.detected)
    {
        ARUCO.detected = true;
        searchReported = false;
        ROS_INFO_STREAM("Guider has been detected");
        sweepComplete = false;
    }

    ARUCO.pose.vector.x = msg->vector.z;
    ARUCO.pose.vector.y = msg->vector.x;
    ARUCO.pose.vector.z = msg->vector.y;

    ARUCO.shortDist = roundf64(sqrt(pow(ARUCO.pose.vector.x, 2) + pow(ARUCO.pose.vector.y, 2)) * 10) / 10;
    ARUCO.thresholdErr = 0.01;

    if (ARUCO.shortDist <= (ARUCO.distanceLimit + ARUCO.thresholdErr) && ARUCO.shortDist >= (ARUCO.distanceLimit - ARUCO.thresholdErr))
    {

        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        if(!ARUCO.reached){
            ROS_INFO_STREAM("Guider stationary");
            ARUCO.reached = true;
        }
    }

    // adjust linear angular velocity
    else
    {
        ARUCO.reached = false;
        if (!objectDetected)
        {
            double error = ARUCO.shortDist - ARUCO.distanceLimit;
            if (ARUCO.shortDist > ARUCO.distanceLimit * 1.5)
            {
                cmd_vel.linear.x = error;
            }
            else if (ARUCO.shortDist < ARUCO.distanceLimit* 0.95)
            {
                cmd_vel.linear.x = -0.3;
            }
            else
                cmd_vel.linear.x = error / 2;



            if (ARUCO.pose.vector.y == 0)
            {
                cmd_vel.angular.z = 0;
            }
            else
                cmd_vel.angular.z = -ARUCO.pose.vector.y;
        }
    }

    cmd_vel_pub.publish(cmd_vel);

    start = ros::Time::now();
}


void FollowTarget::run()
{
    ROS_INFO_STREAM("run function");
    // While loop so that robot is always looking for target.
    while (ros::ok())
    {

        // New Laser scan.
        LaserProcessing laser;

        // Get new laser reading.
        laser.newScan(laser_scan_);

        // Analyse new laser reading.
        laser.reviewLaserReadings();

        // Check if obstacle is blocking robot path.
        if (laser.checkObstacle())
        {
            ROS_INFO_STREAM("OBSTACLED DETECTED IN PATH.");
            stop();
        } else {
            ROS_INFO_STREAM("NO OBSTACLE DETECTED.");
        }

        if (duration > ros::Duration(20.0)){
            start = ros::Time::now();
        }

        duration = ros::Time::now() - start;

        if (duration >= ros::Duration(2.0) && duration <= ros::Duration(8.0) && !ARUCO.detected && !sweepComplete){
            cmd_vel.angular.z = -1.57;
            cmd_vel.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel);
            if(searchReported){
                ROS_INFO_STREAM("Searching for ArUco marker");
                searchReported = true;
            }
        }

        if (duration > ros::Duration(8.0) && duration <= ros::Duration(14.0) && !ARUCO.detected && !sweepComplete){
            cmd_vel.angular.z = 1.57;
            cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel);
        }

        if (duration == ros::Duration(14.0) && !ARUCO.detected && !sweepComplete){
            sweepComplete = true;
            searchReported = false;
            ROS_INFO_STREAM("NO MARKER FOUND. STOPPING SEARCH");
        }

        if (duration >= ros::Duration(3.0) && ARUCO.detected){
            ROS_INFO_STREAM("NO MARKER FOUND, STOPPING OPERATION");
            ARUCO.detected = false;
            sweepComplete = false;
            start = ros::Time::now();
        }
      
    }
}

void FollowTarget::laserCallback(const sensor_msgs::LaserScanPtr &msg)
{
    laser_scan_ = *msg;
}

void FollowTarget::stop()
{

}
