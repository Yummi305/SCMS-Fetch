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

    // PD Controller params
    //ARUCO.distanceLimit = 1.0;
    ARUCO.prev_error = 0.0;
    ARUCO.integral_ = 0.0;
    ARUCO.kp = 0.5; // proportional gain
    ARUCO.kd = 0.1; // derivative gain
    ARUCO.ki = 0.01; // integral gain

    duration = start - start;

    sweepComplete = false;
    //objectDetected = false;
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

    double distance = DistancetoMarker(msg->vector);

    if (!ARUCO.detected)
    {
        ARUCO.detected = true;
        searchReported = false;
        ROS_INFO_STREAM("Guider has been detected");
        sweepComplete = false;
    }

    if (distance > 1.1){
        FollowTarget::followAruco(msg->vector,distance);
    }
    else if (distance < 0.9){
        FollowTarget::moveBackward(distance);
    }
    else FollowTarget::stop();

    start = ros::Time::now();
    
}

double FollowTarget::DistancetoMarker(const geometry_msgs::Vector3 &msg){
    // Fetch starting coords
    ARUCO.robot_x = 0.0;
    ARUCO.robot_y = 0.0;
    ARUCO.robot_z = 0.0;

    //Aruco coords
    //Translate to camera frame of reference (look at rqt_image_view)
    ARUCO.marker_x = msg.z;
    ARUCO.marker_y = msg.x;
    ARUCO.marker_z = msg.y;

    //Euclidean distance formula
    double distance = sqrt(pow(ARUCO.marker_x - ARUCO.robot_x,2)
    +pow(ARUCO.marker_y - ARUCO.robot_y,2)
    +pow(ARUCO.marker_z - ARUCO.robot_z,2));

    return distance;
}

void FollowTarget::followAruco(const geometry_msgs::Vector3 &msg, double distance){

    ARUCO.desired_velocity = FollowTarget::PIDController(distance);

    cmd_vel.linear.x = ARUCO.desired_velocity;

    if(ARUCO.marker_y == 0){
        cmd_vel.angular.z = 0;
    }
    else cmd_vel.angular.z = -ARUCO.marker_y; //inverse direction

    cmd_vel_pub.publish(cmd_vel);
}

void FollowTarget::moveBackward(double distance){

    ARUCO.desired_velocity = FollowTarget::PIDController(distance);

    cmd_vel.linear.x = -ARUCO.desired_velocity;
    cmd_vel_pub.publish(cmd_vel);
}

double FollowTarget::PIDController(double error){

    ARUCO.integral_ += error;
    double p_term = ARUCO.kp * error;
    double d_term = ARUCO.kd * (error - ARUCO.prev_error);
    double i_term = ARUCO.ki * ARUCO.integral_;
    ARUCO.prev_error = error;

    ARUCO.desired_velocity = p_term + d_term;

    return ARUCO.desired_velocity;

}

void FollowTarget::run()
{
    ROS_INFO_STREAM("run function");
    // While loop so that robot is always looking for target.
    while (ros::ok())
    {
        bool object;

        // New Laser scan.
        LaserProcessing laser;

        // Get new laser reading.
        laser.newScan(laser_scan_);

        // Analyse new laser reading.
        laser.reviewLaserReadings();

        object = laser.checkObstacle();

        // Check if obstacle is blocking robot path.
        if (object)
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
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel);
}
