#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing() : euclidean_distance_(0.3f)
{
    ROS_INFO_STREAM("LASER CREATED.");
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan)
{
    laserScan_ = laserScan;
    ROS_INFO_STREAM("NEW SCAN.");
}

void LaserProcessing::reviewLaserReadings()
{
    ROS_INFO_STREAM("looking at laser msgs");
    // Initialise values.
    std::vector<int> laserReadings;
    geometry_msgs::Point conePoint;
    bool validReading = false;

    for (int i = 0; i < laserScan_.ranges.size(); i++)
    {
        // Laser readings outside this range are invalid.
        if (laserScan_.ranges.at(i) < laserScan_.range_min || laserScan_.ranges.at(i) > laserScan_.range_max)
        {
            // End of segment.
            if (validReading)
            {
                    int obstacleReading = laserReadings.at(laserReadings.size() / 2);
                    double distanceInfront = laserScan_.ranges.at(obstacleReading);
                    // Check for obstacle blocking path.
                    ROS_INFO_STREAM("valid reading");
                    if (obstacleReading > 50 && distanceInfront < 5)
                    {
                        obstacle_ = true;
                    }
                // }
                laserReadings.clear();
                validReading = false;
            }
            else
            {
                ROS_INFO_STREAM("NOT VALID READING");
                continue;
            }
        }
        // Process valid readings.
        else
        {
            // Valid reading, continue processing.
            if (validReading)
            {
                double displacement = laserScan_.ranges.at(i) - laserScan_.ranges.at(i - 1);

                // Classify segments based on the 0.3m Euclidean distance between successive cones.
                if (displacement > euclidean_distance_)
                {
                    laserReadings.clear();
                    validReading = false;
                    continue;
                }
                else if (displacement < -euclidean_distance_)
                {
                    if (laserReadings.size() < 15)
                    {
                        laserReadings.push_back(i);
                        conePoint = polarToCart(laserReadings.at(laserReadings.size() / 2));
                        laserReadings.clear();
                        validReading = false;
                    }
                }
                // Continue processing segment.
                else
                {
                    laserReadings.push_back(i);
                    validReading = true;
                }
            }
            // Begin new segment.
            else
            {
                laserReadings.push_back(i);
                validReading = true;
            }
        }
    }
}

geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment * index;
    float range = laserScan_.ranges.at(index);
    // Assign the components of the points
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range * std::cos(angle));
    cart.y = static_cast<double>(range * std::sin(angle));
    cart.z = 0;
    return cart;
}

bool LaserProcessing::checkObstacle()
{
    ROS_INFO_STREAM("CHECKING FOR OBSTACLE");
    // If obstacle in path, update obstacle_ condition.
    if (obstacle_)
    {
        ROS_INFO_STREAM("OBSTACLE");
        return true;
    }
    else
    {
        ROS_INFO_STREAM("NO OBSTACLE");
        return false;
    }
}