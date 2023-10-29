#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include "std_srvs/SetBool.h"

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
  void getConeLocations();


  /*! @brief getGlobalPoints
   *  Convert points from local plane to global plane.
   *  @param cone_points - container of points.
   *  @param pose - current pose of laser
   *  Converts cone points to global points.
   */
  std::vector<geometry_msgs::Point> getGlobalPoints(std::vector<geometry_msgs::Point> cone_points, geometry_msgs::Pose pose);

  /*! @brief localToGlobal
   *  Convert one point from local plane to global plane.
   *  @param localPoint - point in local plane.
   *  @param pose - current pose of laser
   *  Converts given point to global point in the global plane.
   */
  geometry_msgs::Point localToGlobal(geometry_msgs::Point localPoint, geometry_msgs::Pose pose);

  /*! @brief normaliseAngle
   *  Normalises angle given.
   *  @param theta - angle
   */
  double normaliseAngle(double theta);

  /*! @brief Returns the cartesian position of laser reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
  geometry_msgs::Point polarToCart(unsigned int index);

  bool checkObstacle();

private:
  sensor_msgs::LaserScan laserScan_;
  const float cone_distance_;
  std::vector<std::pair<double, double>> cones_;
  bool obstacle_;
  geometry_msgs::Point roadCentre_;
};

#endif // DETECTCABINET_H
