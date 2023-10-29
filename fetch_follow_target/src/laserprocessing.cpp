#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing() : euclidean_distance_(0.3f)
{
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan)
{
    laserScan_ = laserScan;
}

void LaserProcessing::reviewLaserReadings()
{
    // Initialise values.
    std::vector<int> laserReadings;
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
                    if (obstacleReading > 30 && distanceInfront < 0.7)
                    {
                        obstacle_ = true;
                    }
                    else obstacle_ = false;
                // }
                laserReadings.clear();
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