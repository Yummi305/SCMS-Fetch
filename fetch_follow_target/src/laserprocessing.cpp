#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing() : cone_distance_(0.3f)
{
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan)
{
    laserScan_ = laserScan;
}


void LaserProcessing::getConeLocations()
{
    // Initialise values.
    std::vector<int> coneReadings;
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
                // Fewer range readings means that object is likely a cone.
                if (coneReadings.size() < 30)
                {
                    // Convert laser reading from polar coordinate to cartesian position.
                    conePoint = polarToCart(coneReadings.at(coneReadings.size() / 2));
                    // Push into cones_ container.
                    cones_.push_back(std::make_pair(conePoint.x, conePoint.y));
                }
                // Larger range readings means that object is likely an obstacle.
                else
                {
                    int obstacleReading = coneReadings.at(coneReadings.size() / 2);
                    double distanceInfront = laserScan_.ranges.at(obstacleReading);
                    // Check for obstacle blocking path.
                    if (obstacleReading > 50 && distanceInfront < 5)
                    {
                        obstacle_ = true;
                    }
                }
                coneReadings.clear();
                validReading = false;
            }
            else
            {
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
                if (displacement > cone_distance_)
                {
                    coneReadings.clear();
                    validReading = false;
                    continue;
                }
                else if (displacement < -cone_distance_)
                {
                    if (coneReadings.size() < 15)
                    {
                        coneReadings.push_back(i);
                        conePoint = polarToCart(coneReadings.at(coneReadings.size() / 2));
                        cones_.push_back(std::make_pair(conePoint.x, conePoint.y));

                        coneReadings.clear();
                        validReading = false;
                    }
                }
                // Continue processing segment.
                else
                {
                    coneReadings.push_back(i);
                    validReading = true;
                }
            }
            // Begin new segment.
            else
            {
                coneReadings.push_back(i);
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


std::vector<geometry_msgs::Point> LaserProcessing::getGlobalPoints(std::vector<geometry_msgs::Point> cone_points, geometry_msgs::Pose pose)
{
    // Initialise global points container.
    std::vector<geometry_msgs::Point> points;

    // Convert points in local plane to points in global plane and push into global points container.
    for (int i = 0; i < cone_points.size(); ++i)
    {
        geometry_msgs::Point globalPoint = localToGlobal(cone_points.at(i), pose);
        points.push_back(globalPoint);
    }

    // for (int i = 0; i < points.size(); ++i)
    // {
        // ROS_INFO_STREAM("cone " << i << " x,y " << points.at(i).x << "," << points.at(i).y);
    // }

    return points;
}


geometry_msgs::Point LaserProcessing::localToGlobal(geometry_msgs::Point localPoint, geometry_msgs::Pose pose)
{
    geometry_msgs::Point point;
    double yaw = normaliseAngle(tf::getYaw(pose.orientation));

    point.x = (localPoint.x * cos(yaw) - localPoint.y * sin(yaw) + pose.position.x + 3.7 * cos(yaw));
    point.y = (localPoint.x * sin(yaw) + localPoint.y * cos(yaw) + pose.position.y + 3.7 * sin(yaw));

    // ROS_INFO_STREAM("cone points converted to global");
    return point;
}


double LaserProcessing::normaliseAngle(double theta)
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


bool LaserProcessing::checkObstacle()
{
    // If obstacle in path, update obstacle_ condition.
    if (obstacle_)
    {
        return true;
    }
    else
    {
        return false;
    }
}