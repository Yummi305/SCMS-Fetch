#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include "std_msgs/Float64.h"
#include "laserprocessing.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <chrono>
#include <time.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

namespace pfms {

    typedef enum {
      ACKERMAN, /*!< Ackerman based steering ground vehicle */
      QUADCOPTER /*!< Quadcopter */
    } PlatformType; /*!< Platform Types */

    typedef enum {
      IDLE, /*!< Stationary */
      RUNNING, /*!< Executing a motion */
      TAKEOFF, /*!< UAV only, taking off */
      LANDING /*!< UAV only, landing */
    } PlatformStatus; /*!< Platform Status */

}

/*!
 *  \brief     Controller Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2022-04-15
 *  \pre       none
 *  \bug       none reported as of 2022-04-15
 */

class ControllerInterface
{
public:
  ControllerInterface(){};


  /*! @brief 
   *  Non-blocking call in which the thread for the Ackerman class is initialised.
   *  This function is locked by a mutex and has a conditional variable that does not allow this function to run until the atomic bool condition is met.
   *  This conditional variable is unlocked by the runMission function in the Controller Class and is triggered by the request function in the Ackerman Class.
   *  For more details, see the \ref runMission() function. Also, see the \ref request() function.
   *  Function flow:
   *  1. Check if the mission is running. If not, do not continue.
   *    1.2. Take new laser scan to detect cones. For more details, see \ref detectConePair() function. 
   *    1.3. Publish visualisation for cones.
   *  2. Check which mode the program is in. [Basic Mode or Advanced Mode]
   *  3. If in Basic Mode:
   *    3.1 Check if goals have been extracted. If not, extract goals then set goals_extracted_ condition to true so that the provided goals are not set again until the Ackerman has reached all possible goals.
   *    3.2 Check if there are goals left to be reached.
   *    3.3 Check if a goal has been set to be reached. If not, set a goal and update goal_set_ condition to true. This condition prevents the goal from being changed whilst the Ackerman is attempting to reach a goal.
   *    3.4 Attempt to reachGoal. For more details, see \ref reachGoal().
   *    3.5 Check if the Ackerman has reached the current goal and there are goals left to be reached. If so, add a goal_counter_ so that the next goal can be set. Reset reach_goal_ condition.
   *    3.6 Check if the Ackerman is now attempting to reach the final goal that was provided. If so, set goal_left_ condition to false so that the progrm stops as there are no more goals to be reached.
   *  4. If in Advanced Mode:
   *    4.1 Check if the road centre has been set. If not, use laser to determine the road centre based on the cones. For more details, see \ref detectRoadCentre() fucntion.
   *    4.2 Convert road centre to global point and set as goal.
   *    4.3 Publish visulation for road centre goal.
   *    4.4 Set centre_set_ condition to true so that the Ackerman's goal is not changed while it is attempting to reach it.
   *    4.5 Since road centre goal has been set, attempt to reach it.
   *    4.6 If the road centre goal has been reached, reset conditions to allow the next road centre goal to be set.
   *  5. Set the rate limiter to sleep.
   */   
  virtual void run(void) = 0;


  /*! @brief status
   *  Checks current platform status and returns RUNNING or IDLE.
   */
  virtual pfms::PlatformStatus status(void) = 0;


  /**
  Setter for goals
  @param goals
  @return all goal reachable, in order supplied
  */
  virtual void setGoals(std::vector<geometry_msgs::Pose> pose_container) = 0;


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
                                        double& distance,
                                        double& time,
                                        nav_msgs::Odometry& estimatedGoalPose) = 0;

  /**
  Getter for pltform type
  @return PlatformType
  */
  virtual pfms::PlatformType getPlatformType(void) = 0;

  /**
  Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
  virtual bool setTolerance(double tolerance) = 0;

  /**
  returns current odometry information
  @return odometry - current odometry
  */
  virtual nav_msgs::Odometry getOdometry(void) = 0;

protected:

};

#endif // CONTROLLERINTERFACE_H
