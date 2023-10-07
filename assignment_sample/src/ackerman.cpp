#include "ackerman.h"
#include <cmath>

Ackerman::Ackerman(ros::NodeHandle nh) : brake_(0.0)
{
  // Mission mode
  ros::NodeHandle mode("~");
  mode.param<bool>("ADVANCED", advance_mode_, true);

  // Initialsie values
  type_ = pfms::PlatformType::ACKERMAN;
  platform_status_ = false;

  // ROS Subscriber
  goals_subscribe_ = nh_.subscribe("/orange/goals", 1000, &Ackerman::goalsCallback, this);

  // ROS Publisher
  brake_publish_ = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd", 3, false);
  steering_publish_ = nh_.advertise<std_msgs::Float64>("/orange/steering_cmd", 3, false);
  throttle_publish_ = nh_.advertise<std_msgs::Float64>("/orange/throttle_cmd", 3, false);

  // ROS Service
  request_service_ = nh_.advertiseService("/orange/mission", &Ackerman::request, this);

  // visualisation initialise
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1000, false);
  cones_pub_ = nh_.advertise<geometry_msgs::PoseArray>("orange/cone", 1000);

  // Initialise thread and push into container.
  threads_.push_back(std::thread(&Ackerman::run, this));

  ROS_INFO_STREAM("ackerman created");
}

// Default destructor when program ends.
Ackerman::~Ackerman()
{
  if (laserProcessingPtr_ != nullptr)
  {
    delete laserProcessingPtr_;
  }
}

bool Ackerman::request(std_srvs::SetBool::Request &req,
                       std_srvs::SetBool::Response &res)
{
  ROS_INFO_STREAM("SERVICE REQUEST");
  ROS_INFO_STREAM("Requested:" << req.data);

  if (req.data)
  {
    res.success = true;
    ROS_INFO_STREAM("Running requested");
    platform_status_ = true;
    missionRun_ = true;
    this->Controller::runMission();
  }
  else
  {
    ROS_INFO_STREAM("Not currently running request");
    missionRun_ = false;
  }

  if (!advance_mode_)
  {
    res.message = "Mode = Basic. Start";
  }
  else
  {
    res.message = "Mode = Advanced. Start";
  }

  return true; // We return true to indicate the service call sucseeded (your responce should indicate a value)
}

void Ackerman::run(void)
{
  std::unique_lock<std::mutex> lck(mtx_);

  // We wait for the convar to release us, unless data is already ready
  cv_.wait(lck, [&]()
           { return atomic_cond_ == true; });
  lck.unlock();

  // ROS_INFO_STREAM("we have waited");

  while (ros::ok())
  {
    // If Request is false, we don't do any action.
    if (!missionRun_)
    {
      // ROS_INFO_STREAM("Not running mission.");
      platform_status_ = false;
      stopCar();
      rate_limiter_.sleep();
      continue;
    }

    // New Laser scan
    LaserProcessing laser;

    // Process laser readings
    geometry_msgs::Pose pose = odo_.pose.pose;
    laser.newScan(laser_scan_);

    // Get cone points
    std::vector<geometry_msgs::Point> cone_pair = laser.detectConePair();

    // Convert detected cones to global points
    std::vector<geometry_msgs::Point> global_cone_pair = laser.getGlobalPoints(cone_pair, pose);

    // Publish cones detected by laser
    auto cone_container = createVizMarkers(global_cone_pair, cone_marker_);
    cones_pub_.publish(cone_container);
    viz_pub_.publish(cone_container);

    // Check if there is an object that is blocking the path of Ackerman.
    if (laser.checkObstacle())
    {
      ROS_INFO_STREAM("Stopping the mission due to obstacle...");
      missionRun_ = false;
    }

    if (!advance_mode_)
    {
      // ROS_INFO_STREAM("BASIC MODE");
      // If provided goals have not been extracted for the first time, do so.
      if (!goals_extracted_)
      {
        // ROS_INFO_STREAM("Setting goals for first time");
        // ROS_INFO_STREAM("Pose container has " << pose_container_.size() << " poses.");
        setGoals(pose_container_);
        goals_extracted_ = true;
      }

      // Check if we have been given goals. If not, just continue driving on track.
      if (goals_left_)
      {
        // ROS_INFO_STREAM("current goal " << goal_counter_ << ": x,y = " << goals_.at(goal_counter_).x << "," << goals_.at(goal_counter_).y);

        // Set goal if none already.
        if (!goal_set_)
        {
          // If goal is valid, set it.
          if (setGoal(goals_.at(goal_counter_)))
          {
            goal_set_ = true;
          }
        }

        // If goal has been set, attempt to reach.
        if (goal_set_)
        {
          // Reach Goal
          reachGoal(goals_.at(goal_counter_));

          // If goal has been reached and there are goals that have not been reached, add a counter.
          if (reach_goal_ && goals_left_ && goal_counter_ < goals_.size())
          {
            // Add counter
            goal_counter_++;
            // ROS_INFO_STREAM("adding to goal counter, now it is " << goal_counter_);
            reach_goal_ = false;
          }

          // Check if the Ackerman is now attempting to reach its final given goal.
          if (goal_counter_ == goals_.size())
          {
            ROS_INFO_STREAM("All provided goals reached.");
            goals_left_ = false;
          }
        }
      }
      else
      {
        ROS_INFO_STREAM("No goals left. Mission stopped");
        missionRun_ = false;
      }
    }
    else
    {
      // ROS_INFO_STREAM("ADVANCED MODE");
      // If provided goals have not been extracted for the first time, do so.
      if (!goals_extracted_)
      {
        // ROS_INFO_STREAM("Setting goals for first time");
        // ROS_INFO_STREAM("Pose container has " << pose_container_.size() << " poses.");
        setGoals(pose_container_);
        goals_extracted_ = true;
      }

      // Check if we have been given goals. If not, just continue driving on track.
      if (goals_left_)
      {
        // ROS_INFO_STREAM("current goal " << goal_counter_ << ": x,y = " << goals_.at(goal_counter_).x << "," << goals_.at(goal_counter_).y);

        // Set goal if none already.
        if (!goal_set_)
        {
          // If goal is valid, set it.
          if (setGoal(goals_.at(goal_counter_)))
          {
            goal_set_ = true;
          }
        }

        // If goal has been set, attempt to reach.
        if (goal_set_)
        {
          // Reach Goal
          reachGoal(goals_.at(goal_counter_));

          // If goal has been reached and there are goals that have not been reached, add a counter.
          if (reach_goal_ && goals_left_ && goal_counter_ < goals_.size())
          {
            // Add counter
            goal_counter_++;
            // ROS_INFO_STREAM("adding to goal counter, now it is " << goal_counter_);
            reach_goal_ = false;
          }

          // Check if the Ackerman is now attempting to reach its final given goal.
          if (goal_counter_ == goals_.size())
          {
            ROS_INFO_STREAM("All provided goals reached.");
            goals_left_ = false;
          }
        }
      }
      else
      {
        // ROS_INFO_STREAM("Continuing to follow track as no goals have been specified.");
        // Condition so that goal does not change mid Ackerman travel.
        if (!centre_set_)
        {
          // Detect road centre and set as desired point.
          geometry_msgs::Pose pose = odo_.pose.pose;
          geometry_msgs::Point road_centre = laser.getRoadCentre();

          // Convert road centre to global point and set as goal.
          goal_ = laser.localToGlobal(road_centre, pose);

          // Check if desired point is reachable.
          if (setGoal(goal_))
          {
            // Set goal.
            ROS_INFO_STREAM("Road centre set as goal.");
            // Publish visualisation for goal.
            goal_marker_points_.push_back(goal_);
            auto goal_vis_container = createVizMarkers(goal_marker_points_, goal_marker_);
            viz_pub_.publish(goal_vis_container);
            // Update centre_set_ condition so that the goal is not updated mid travel.
            centre_set_ = true;
          }
        }

        // If centre has been set as goal, attempt to reach it.
        if (centre_set_)
        {
          // Reach goal.
          reachGoal(goal_);
        }

        // If goal has been reached. We need to reset flag so we can find new goal.
        if (reach_goal_)
        {
          // Reset the goal.
          reach_goal_ = false;
          centre_set_ = false;
          centre_counter_++;
          // ROS_INFO_STREAM("road centre flags reset.");
        }
      }
    }
  }
  rate_limiter_.sleep();
}

void Ackerman::reachGoal(geometry_msgs::Point goal)
{
  // Set Steering for ackerman only once when goal has been set.
  if (!steering_set_for_goal_)
  {
    bool took_took_long = false;

    getOdometry();
    double distance1 = 0;
    double t2g = 0;
    nav_msgs::Odometry estimate1;

    // Update steering.
    update_steering_ = true;
    checkOriginToDestination(odo_, goal, distance1, t2g, estimate1);
    update_steering_ = false;

    // Steering has been set. Do not allow it to be changed until goal has been reached.
    steering_set_for_goal_ = true;
    // ROS_INFO_STREAM("Steering updated.");
  }

  // Attempt to reach goal if it has not yet been reached.
  if (!reach_goal_)
  {
    // ROS_INFO_STREAM("Attempting to reach goal: " << goal_counter_);

    // Check Ackerman's speed and limit it if necessary.
    limitSpeed();

    // Send commands to Ackerman
    sendCmd(brake_, steering_, 0.1);

    // Check if goal has been reached.
    if (checkGoalReached(goal))
    {
      ROS_INFO_STREAM("Goal " << goal_counter_ << " has been reached.");
      reach_goal_ = true;
    }
  }
}

bool Ackerman::checkGoalReached(geometry_msgs::Point goal)
{
  // get current odometry
  getOdometry();

  // ROS_INFO_STREAM("odo x,y: " << odo_.pose.pose.position.x << "," << odo_.pose.pose.position.y);
  double displacement = std::hypot(odo_.pose.pose.position.x - goal.x, odo_.pose.pose.position.y - goal.y);

  // ROS_INFO_STREAM("checking if goal " << goal_counter_ << " has been reached. Displacement from goal: " << displacement);

  // Check if Ackerman is currently within tolerance.
  if (displacement <= tolerance_)
  {
    // ROS_INFO_STREAM(" Goal Reached! ");
    stopCar();
    goal_set_ = false;
    steering_set_for_goal_ = false;
    return true;
  }
  else
  {
    // ROS_INFO_STREAM("goal not yet reached");
    return false;
  }
}

bool Ackerman::checkOriginToDestination(nav_msgs::Odometry origin,
                                        geometry_msgs::Point goal,
                                        double &distance,
                                        double &time,
                                        nav_msgs::Odometry &estimatedGoalPose)
{

  // Calculate the unit yaw vector.
  geometry_msgs::Point yaw_vector;
  yaw_vector.x = cos(tf::getYaw(origin.pose.pose.orientation));
  yaw_vector.y = sin(tf::getYaw(origin.pose.pose.orientation));
  yaw_vector.z = 0;

  // Calculate the displacement vector between the current position and the goal position.
  geometry_msgs::Point displacement_vector;
  displacement_vector.x = goal.x - origin.pose.pose.position.x;
  displacement_vector.y = goal.y - origin.pose.pose.position.y;

  // Calculate the straight line distance between the current position and the goal position.
  double displacement_ = sqrt(pow(displacement_vector.x, 2) + pow(displacement_vector.y, 2));

  // Calculate the dot product between the unit yaw vector and the displacement vector.
  double dot = (yaw_vector.x * displacement_vector.x) + (yaw_vector.y * displacement_vector.y);

  // Calculate the determinant between the unit yaw vector and the displacement vector.
  double determinant = (yaw_vector.x * displacement_vector.y) - (yaw_vector.y * displacement_vector.x);

  // Calculate half the wheel angle (alpha).
  double alpha_ = atan2(determinant, dot);

  // Calculate the steering angle (delta).
  double delta_ = atan((2 * wheelbase_ * sin(alpha_)) / displacement_);

  if (update_steering_)
    steering_ = delta_ * steering_ratio_;

  // Calculate the turning radius.
  double radius_ = abs(displacement_ / (2 * sin(alpha_)));

  // Calculate the arc distance.
  distance = abs(radius_ * 2 * alpha_);
  distance_ = distance;

  // Set GoalPose position.
  estimatedGoalPose.pose.pose.position.x = goal.x;
  estimatedGoalPose.pose.pose.position.y = goal.y;

  // Calculate x and y inverse.
  double x_inverse = (displacement_vector.x * cos(-tf::getYaw(origin.pose.pose.orientation))) - (displacement_vector.y * sin(-tf::getYaw(origin.pose.pose.orientation)));
  double y_inverse = (displacement_vector.x * sin(-tf::getYaw(origin.pose.pose.orientation))) + (displacement_vector.y * cos(-tf::getYaw(origin.pose.pose.orientation)));

  // Calculate yaw.
  tf::Quaternion temp_quaternion;
  temp_quaternion.setRPY(0, 0, normaliseAngle(2 * atan2(y_inverse, x_inverse)) + tf::getYaw(origin.pose.pose.orientation));
  geometry_msgs::Quaternion orientation_temp;
  // Translate yaw into quaternion.
  tf::quaternionTFToMsg(temp_quaternion, orientation_temp);
  // Set orientation for estimated goal pose.
  estimatedGoalPose.pose.pose.orientation = orientation_temp;

  // Calculate time based on arc length.
  time = distance / target_speed_;

  // Check if steering angle required to reach the goal is possible.
  if (abs(delta_) <= max_steering_angle_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Ackerman::sendCmd(double brake, double steering, double throttle)
{
  std_msgs::Float64 value;
  value.data = brake;
  brake_publish_.publish(value);
  value.data = steering;
  steering_publish_.publish(value);
  value.data = throttle;
  throttle_publish_.publish(value);
  // ROS_INFO_STREAM("sending command to ackerman");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Ackerman::stopCar()
{
  getOdometry();
  velocity_ = std::hypot(odo_.twist.twist.linear.x, odo_.twist.twist.linear.y);
  while (velocity_ > 0.5)
  {
    velocity_ = std::hypot(odo_.twist.twist.linear.x, odo_.twist.twist.linear.y);
    // ROS_INFO_STREAM("Slowing down Ackerman... velocity: " << velocity_);
    sendCmd(8000, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Ackerman::goalsCallback(const geometry_msgs::PoseArray &msg)
{
  std::unique_lock<std::mutex> lck(mtx_);
  // Extract poses from PoseArray
  geometry_msgs::PoseArray temp_msg;
  temp_msg = msg;
  pose_container_ = temp_msg.poses;
  unsigned int n = 0;
  for (auto pos : pose_container_)
  {
    ROS_INFO_STREAM("A goal has been set at x;y: " << pos.position.x << ";" << pos.position.y);
    n++;
  }
  ROS_INFO_STREAM("number of goals set: " << n);
}

void Ackerman::limitSpeed()
{
  getOdometry();
  velocity_ = std::hypot(odo_.twist.twist.linear.x, odo_.twist.twist.linear.y);

  if (velocity_ > 2.0)
  {
    brake_ = 500;
  }
  else
  {
    brake_ = 0;
  }
}
