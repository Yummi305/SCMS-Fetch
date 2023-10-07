#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "controllerinterface.h"

using namespace pfms;

/*!
 *  \brief     Controller Class
 *  \details
 *  This class has inhereted functions from the Controller Interface Class which defines the functionality of this program.
 *  This is the base class for the Ackerman and Quadcopter Classes. It has all pure virtual functions from the Controller Interface Class. Functions that can be accessed by any platform should be implemented here.
 *  Any virtual functions that are to be implemented within the Ackerman or Quadcopter only have been created as pure virtual so that Ackerman and Quadcopter Classes must implement them.
 *  This class interacts with the Mission Class which facilitates the mission of both platforms. Please see the comments below.
 *  \note
 *  All functions without comments below have an explanation in the relevant Interface Class.
 *  \author    Dennis Tuan Nguyen
 *  \version   1.01-2
 *  \date      2023-04-17
 *  \pre       none
 *  \bug       none reported as of 2023-05-02
 */

class Controller : public ControllerInterface
{
public:
  /**
  Default contrustor.
  */
  Controller();

  /**
  Destructor.
  */
  ~Controller();

  // Pure virtual functions to ensure implementation in derived classes.

  /*! @brief
   *  @param[in] index -
   */
  virtual void reachGoal(geometry_msgs::Point goal) = 0;

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(nav_msgs::Odometry origin,
                                        geometry_msgs::Point goal,
                                        double &distance,
                                        double &time,
                                        nav_msgs::Odometry &estimatedGoalPose) = 0;

  /*! @brief
   *  @param[in] index -
   */
  virtual void stopCar() = 0;

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
  virtual void run(void) = 0;

  /**
   * @brief setGoal
   * Check if goal is reachable. If so, set goal_set_ condition to true.
   * @param goal
   * @return goal reachable
  */
  bool setGoal(geometry_msgs::Point goal);

  /*! @brief setGoals
   *  Recieves container of poses and converts them into goal points which are stored in the goals_ container.
   *  @param pose_container 
   */
  void setGoals(std::vector<geometry_msgs::Pose> pose_container);

  /*! @brief setTolerance
   *  Checks if tolerance is valid and sets as tolerance_.
   *  @param tolerance
   */
  bool setTolerance(double tolerance);

  /*! @brief status
   *  Checks current platform status and returns RUNNING or IDLE.
   */
  pfms::PlatformStatus status(void);

  /*! @brief getOdometry
   *  Get current Odometry.
   */
  nav_msgs::Odometry getOdometry(void);

  /*! @brief getPlatformType
   *  Check platform type and returns it. E.g. Ackerman
   */
  pfms::PlatformType getPlatformType(void);

  /**
  @brief
  * controller in reaching goals - non blocking call.
  * This function checks if the goals have been set. Then the convar notifies the respective \ref Ackerman::reachGoal and Quadcopter::reachGoal functions.
  */
  void runMission(void);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
  void seperateThread();

  /*! @brief Odometry Callback
   *
   *  @param msg - Odometry message.
   *  Callback function for odometry.
   *  @note This function and the declaration are ROS specific
   */
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);

  /*! @brief LaserScan Callback
   *
   *  @param msg - Laser message.
   *  Callback function for Laser.
   *  @note This function and the declaration are ROS specific
   */
  void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief
   * Normalises angle between -PI to PI, can only handle angles between -2PI to 4PI.
   * @param theta which is an angle in radians.
   * @return normalised angle in radians.
   */
  double normaliseAngle(double theta);

  /*!
   * \brief createConesMarker
   *  Takes in location and desired type of cone. Creates markers and pushes them into container so that they can be published for visualisation in RViz.
   * @param points - location of marker.
   * @param type - Marker type: Cones, Goal, etc.
   */
  visualization_msgs::MarkerArray createVizMarkers(std::vector<geometry_msgs::Point> points, visualization_msgs::Marker type);


protected:
  // Protected variables.
  std::vector<geometry_msgs::Point> goals_;
  double tolerance_;
  double obstacle_tolerance_;
  geometry_msgs::Point goal_;
  pfms::PlatformType type_;
  bool platform_status_;
  nav_msgs::Odometry odo_;
  double distance_;
  bool goal_set_;
  bool steering_set_for_goal_;
  std::vector<geometry_msgs::Pose> pose_container_;
  bool goals_extracted_;
  bool missionRun_;
  bool advance_mode_;
  bool goals_left_;
  double velocity_;
  long id_;
  unsigned int goal_counter_;
  visualization_msgs::Marker goal_marker_;
  visualization_msgs::Marker cone_marker_;
  std::vector<geometry_msgs::Point> cones_;
  geometry_msgs::Point road_;
  std::vector<geometry_msgs::Point> roads_;
  visualization_msgs::MarkerArray markers_;

  // Laser and range
  sensor_msgs::LaserScan laser_scan_;

  // Threading and conditions
  std::vector<std::thread> threads_;
  std::thread *thread_;
  std::mutex mtx_;
  std::mutex rangeMtx_;
  std::mutex odoMtx_;
  std::condition_variable cv_;
  std::atomic<bool> atomic_cond_;
  ros::Rate rate_limiter_;

  // ROS Subscriber
  ros::Subscriber odometry_subscribe_;
  ros::Subscriber laser_subscribe_;
  ros::Subscriber range_subscribe_;
  ros::Subscriber goal_subscribe_;
  ros::Subscriber goals_subscribe_;

  // ROS Service
  ros::ServiceServer request_service_;

  // ROS Node
  ros::NodeHandle nh_;

  // ROS Publisher
  ros::Publisher viz_pub_;
  ros::Publisher cones_pub_;
};

#endif // CONTROLLER_H
