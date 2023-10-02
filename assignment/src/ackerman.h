#ifndef ACKERMAN_H
#define ACKERMAN_H
#include "controller.h"

/*!
 *  \brief     Ackerman Class
 *  \details
 *  This class inherets from the Controller Class and implements all pure virtual functions from that Class.
 *  This class functions in the class are intended to facilitate the movement of the Ackerman platform.
 *  Please read the comments below to see how each function interacts with each other.
 *  \note
 *  All functions without comments below have an explanation in the relevant Interface Class.
 *  \author    Dennis Tuan Nguyen
 *  \version   1.01-2
 *  \date      2023-04-17
 *  \pre       none
 *  \bug       none reported as of 2023-05-02
 */

class Ackerman : public Controller
{
public:
  /**
   * @brief Ackerman Constructor
   * Default constructors should set all sensor attributes to a default value.
   */
  Ackerman(ros::NodeHandle nh);

  /**
   * @brief Ackerman Destructor
   * Default destructor. Thread is joined once program has ended.
   * Pipe and thread also deleted once program has ended.
   */
  ~Ackerman();

  /*! @brief checkOriginToDestination
   *  Checks whether the platform can travel between origin and destination
   *  @param[in] origin The origin pose, specified as odometry for the platform
   *  @param[in] destination The destination point for the platform
   *  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
   *  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
   *  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
   *  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(nav_msgs::Odometry origin,
                                geometry_msgs::Point goal,
                                double &distance,
                                double &time,
                                nav_msgs::Odometry &estimatedGoalPose);

  /*! @brief run
   *  Non-blocking call in which the thread for the Ackerman class is initialised. This function checks what mode the program is in and decides what the Ackerman should do.
   *  This function is locked by a mutex and has a conditional variable that does not allow this function to run until the atomic bool condition is met.
   *  This conditional variable is unlocked by the runMission function in the Controller Class and is triggered by the request function in the Ackerman Class.
   *  For more details, see the \ref runMission() function. Also, see the \ref request() function.
   *  Function flow:
   *  1. Check if the mission is running. If not, do not continue.
   *    1.2. Take new laser scan to detect cones. For more details, see \ref detectConePair() function.
   *    1.3. Publish visualisation for cones.
   *    1.4. Laser check if there is an obstacle in the way. If so, stop mission.
   *  2. Check which mode the program is in. [Basic Mode or Advanced Mode]
   *  3. If in Basic Mode:
   *    3.1 Check if goals have been extracted. If not, extract goals then set goals_extracted_ condition to true so that the provided goals are not set again until the Ackerman has reached all possible goals.
   *    3.2 Check if there are goals left to be reached.
   *    3.3 Check if a goal has been set to be reached. If not, set a goal and update goal_set_ condition to true. This condition prevents the goal from being changed whilst the Ackerman is attempting to reach a goal.
   *    3.4 Attempt to reachGoal. For more details, see \ref reachGoal().
   *    3.5 Check if the Ackerman has reached the current goal and there are goals left to be reached. If so, add a goal_counter_ so that the next goal can be set. Reset reach_goal_ condition.
   *    3.6 Check if the Ackerman is now attempting to reach the final goal that was provided. If so, set goal_left_ condition to false so that the progrm stops as there are no more goals to be reached.
   *  4. If in Advanced Mode:
   *    4.0 If goals given, operate the same as Basic Mode. If no goals provided or no goals left, continue with below.
   *    4.1 Check if the road centre has been set. If not, use laser to determine the road centre based on the cones. For more details, see \ref detectRoadCentre() fucntion.
   *    4.2 Convert road centre to global point and set as goal.
   *    4.3 Publish visulation for road centre goal.
   *    4.4 Set centre_set_ condition to true so that the Ackerman's goal is not changed while it is attempting to reach it.
   *    4.5 Since road centre goal has been set, attempt to reach it.
   *    4.6 If the road centre goal has been reached, reset conditions to allow the next road centre goal to be set.
   *  5. Set the rate limiter to sleep.
   */
  void run(void);

  /**
   * @brief reachGoal
   * The reachGoal function is intended to facilitate movement of the Ackerman platform towards the accepted goal.
   * @param goal the current goal that has been set.
   * 1. The Ackerman checks if the steering has been calculated to reach the goal. If it has, the condition is set to true so that the steering is not updated mid travel.
   *  For more details. See \ref checkOriginToDestination() function.
   * 2. If the goal has not been reached, send arguments to the Ackerman so that it can drive towards the goal.
   *  For more details. See \ref sendCmd() function.
   * 3. Here, the speed is also limited to 2 so that the Ackerman's movement can be easily controlled.
   * 4. It is then checked if the goal has been reached. If so, set reach_goal_ condition to true so that the Ackerman can move onto the next goal.
   */
  void reachGoal(geometry_msgs::Point goal);

  /**
   * @brief sendCmd
   * Accepts arguments that will control how the Ackerman platform moves. These arguments are sent to ROS via the pipe to interact with the Ackerman platform.
   * @param brake the braking force to be applied when slowing down the Ackerman.
   * @param steering_ the steering angle to be applied to the Ackerman when reaching a goal.
   * This value is calculated when determining if the goal can be reached.
   * This value should be constant when progressing towards a goal due to the Ackerman steering model.
   * @param throttle the throttle to be applied to the Ackerman when reaching a goal.
   */
  void sendCmd(double brake, double steering, double throttle);

  /*! @brief request
   *
   *  @param req The request
   *  @param res The responce
   *
   *  @return bool - Will return true if the request has sucseeded
   */
  bool request(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res);

  /*!
   * \brief checkGoalReached
   *  Checks if the Ackerman is within the goal considering the tolerance set.
   * @param goal current goal set.
   * \return bool - Will return true if the Ackerman has reached the goal.
   */
  bool checkGoalReached(geometry_msgs::Point goal);

  /*!
   * \brief stopCar
   * Check the speed of the Ackerman. 
   * Send arguments to brake in order to stop the Ackerman.
   */
  void stopCar();

  /*!
   * \brief limitSpeed
   *  Check current speed of Ackerman.
   *  If speed above 2, set brake to 500.
   *  If speed below 2, set brake to 0.
   *  When the braking_ is sent, it uses the appropriate value.
   *  For more details, see \ref sendCmd function.
   */
  void limitSpeed();

  /*!
   * \brief goalsCallback
   * Callback function for goals.
   *  @note This function and the declaration are ROS specific
   */
  void goalsCallback(const geometry_msgs::PoseArray &msg);

private:
  // Ackerman movement related variables.
  static constexpr double steering_ratio_ = 17.3;
  static constexpr double lock_to_lock_revs_ = 3.2;
  static constexpr double track_width_ = 1.638;
  static constexpr double wheel_radius_ = 0.36;
  static constexpr double wheelbase_ = 2.65;
  static constexpr double target_speed_ = 1.45; // 2.91 at 0.2 throttle
  double max_steering_angle_ = M_PI * (lock_to_lock_revs_ / steering_ratio_);
  double steering_;
  unsigned int centre_counter_;

  // Ackerman related variables.
  geometry_msgs::Point goal_;
  std::vector<geometry_msgs::Point> goal_marker_points_;
  std::vector<double> times;
  double sum_distance_travelled;
  double sum_estimated_distance_travelled;
  double time_;
  double current_dist_to_goal;
  double arcLength_;
  double distance2goal_;
  bool reach_goal_;
  bool centre_set_;
  bool update_steering_ = false;
  double goal_distance_travelled_;
  double prev_dist_trav_;
  double goal_time_past_;
  double time_since_leaving_prev_location_;
  unsigned long cmd_pipe_seq_;
  double brake_;
  // Mutex
  std::mutex laser_mtx_; //!< Mutex associated with laser

  // Laser data
  LaserProcessing *laserProcessingPtr_;

  // ROS Publisher
  ros::Publisher brake_publish_;
  ros::Publisher throttle_publish_;
  ros::Publisher steering_publish_;
};

#endif // ACKERMAN_H
