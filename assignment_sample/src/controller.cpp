#include "controller.h"

Controller::Controller() : missionRun_(false),
                           goal_counter_(0),
                           goals_extracted_(false),
                           goals_left_(false),
                           rate_limiter_(5.0),
                           steering_set_for_goal_(false),
                           tolerance_(0.5),
                           obstacle_tolerance_(2.5)
{
  ROS_INFO_STREAM("creating constructor.");

  // ROS Subscriber
  odometry_subscribe_ = nh_.subscribe("ugv_odom", 1000, &Controller::odomCallback, this);
  laser_subscribe_ = nh_.subscribe("orange/laser/scan", 100, &Controller::laserCallback, this);

  // Initialise Markers.
  goal_marker_.type = visualization_msgs::Marker::CUBE;
  cone_marker_.type = visualization_msgs::Marker::CYLINDER;
}

Controller::~Controller()
{
  // Join threads together.
  for (auto &thread : threads_)
    thread.join();
}

void Controller::runMission()
{
  std::unique_lock<std::mutex> lck(mtx_);
  missionRun_ = true;
  atomic_cond_ = true;
  mtx_.unlock();
  cv_.notify_all();
}

void Controller::setGoals(std::vector<geometry_msgs::Pose> pose_container)
{
  std::unique_lock<std::mutex> lck(mtx_);
  // Loop through all poses
  if (pose_container.size() == 0)
  {
    goals_left_ = false;
    ROS_INFO_STREAM("Goals not set. We have been provided with no goals");
  }
  else
  {
    for (int i = 0; i < pose_container.size(); ++i)
    {
      geometry_msgs::Point curr_goal;
      curr_goal.x = pose_container.at(i).position.x;
      curr_goal.y = pose_container.at(i).position.y;
      curr_goal.z = 0;
      goals_.push_back(curr_goal);
    }
    goals_left_ = true;
    ROS_INFO_STREAM("Goals set. We have been provided with " << goals_.size() << " goals.");
  }
}

pfms::PlatformType Controller::getPlatformType(void)
{
  return type_;
}

bool Controller::setTolerance(double tolerance)
{
  // verify if tolerance input from user is valid. tolerances cannot be negative.
  if (tolerance >= 0.0)
  {
    tolerance_ = tolerance;
    return true;
  }
  else
  {
    return false;
  }
}

double Controller::normaliseAngle(double theta)
{
  // If angle is out of range, normalise it.
  if (theta > (2 * M_PI))
    theta = theta - (2 * M_PI);
  else if (theta < 0)
    theta = theta + (2 * M_PI);

  if (theta > M_PI)
  {
    theta = -((2 * M_PI) - theta);
  }

  return theta;
}

pfms::PlatformStatus Controller::status(void)
{
  if (platform_status_)
  {
    return pfms::PlatformStatus::RUNNING;
  }
  else
  {
    return pfms::PlatformStatus::IDLE;
  }
}

nav_msgs::Odometry Controller::getOdometry(void)
{
  return odo_;
}

bool Controller::setGoal(geometry_msgs::Point goal)
{
  std::unique_lock<std::mutex> lock_odo(mtx_);
  // ROS_INFO_STREAM("Checking if goal " << goal_counter_ << " is reachable.");
  // Reset goal set condition as we are setting a new goal.
  goal_set_ = false;
  // Get current odometry.
  nav_msgs::Odometry origin = getOdometry();
  geometry_msgs::Point point = goal;
  double time = 0;
  double distance = 0;
  bool REACHABLE = checkOriginToDestination(origin, point, distance, time, origin);
  // Check if the destination is reachable.
  if (REACHABLE)
  {
    if (advance_mode_)
    {
      ROS_INFO_STREAM("Platform can reach road centre.");
    }
    else
    {
      ROS_INFO_STREAM("Platform can reach Goal " << goal_counter_ << ".");
    }
    goal_set_ = true;
    return true;
  }
  else
  {
    if (advance_mode_)
    {
      ROS_INFO_STREAM("Platform cannot reach road centre.");
    }
    else
    {
      ROS_INFO_STREAM("Platform cannot reach Goal " << goal_counter_ << ".");
    }
    goal_set_ = false;
    return false;
  }
}

void Controller::seperateThread()
{
  /**
   * The below loop runs until ros is shutdown
   */

  //! THINK : What rate shouls we run this at? What does this limiter do?
  ros::Rate rate_limiter(5.0);
  while (ros::ok())
  {

    rate_limiter.sleep();
  }
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::unique_lock<std::mutex> lck(odoMtx_);
  odo_ = *msg;
  // ROS_INFO_STREAM("ODOM CALLBACK: odo x,y: " << odo_.pose.pose.position.x << "," << odo_.pose.pose.position.y);
}

void Controller::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  laser_scan_ = *msg;
}

visualization_msgs::MarkerArray Controller::createVizMarkers(std::vector<geometry_msgs::Point> points, visualization_msgs::Marker type)
{

  visualization_msgs::MarkerArray markerContainer;
  for (auto pt : points)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    if (type.type == visualization_msgs::Marker::CYLINDER)
    {
      marker.lifetime = ros::Duration(0);
      marker.ns = "cones";
      marker.id = id_++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.5;

      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 300;
      marker.color.a = 1.0;
    }
    else if (type.type == visualization_msgs::Marker::CUBE)
    {
      marker.lifetime = ros::Duration(30);
      marker.ns = "goal";
      marker.id = id_++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      std_msgs::ColorRGBA color;
      marker.color.a = 0.5;
      marker.color.r = 230.0 / 255.0;
      marker.color.g = 230.0 / 255.0;
      marker.color.b = 250.0 / 255.0;
    }

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    markerContainer.markers.push_back(marker);
  }
  return markerContainer;
}