#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"

class LaserProcessing
{
public:
  /*! @brief LaserProcessing constructor
   */
  LaserProcessing();

  /*! @brief newScan
   *  @param[in] laserScan  - laserScan to process.
   */
  void newScan(sensor_msgs::LaserScan laserScan);

  /*!
   *  @brief getConeLocations
   *  @note Segments are formed by high intensity readings.
   * A segment is a sequence (consecutive) high intensity readings that are less than 0.3m
   *
   * @return the number of segments in the current laser scan
   */
  void reviewLaserReadings();

  bool checkObstacle();

private:
  sensor_msgs::LaserScan laserScan_;
  const float euclidean_distance_;
  bool obstacle_;
};

#endif // DETECTCABINET_H
