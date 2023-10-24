#include "../include/laserScan.h"

static const int laserFOV = 140;
static const double laserLimit = 0.35;
static const double radFetch = 0.257;

LaserDetect::LaserDectect(){}

bool LaserDetect::ObjectDetect(sensor_msgs::LaserScan::ConstPtr laserScan){

    double laserReading = laserScan->range_max;                           // get max reading
    int rangeStart = (laserScan->ranges.size() / 2) - (laserFOV / 2);     // consider star of FOV
    int rangeEnd = (laserScan->ranges.size() / 2) + (laserFOV / 2);       //consider end of FOV
    for (unsigned int i = rangeStart; i <= rangeEnd; i++)
    {
        if (laserScan->ranges.at(i) < laserReading)
        {
            laserReading = laserScan->ranges.at(i); // Store laser reading
        }
    }

    if (laserReading <= laserLimit + radFetch) // check if condition is valid
    {
        return true;
    }
    else
    {
        return false;
    }

}

double LaserDetect::LaserReading(sensor_msgs::LaserScan::ConstPtr laserScan){

    int rangeStart = (laserScan->ranges.size() / 2) - (laserFOV / 2);
    int rangeEnd = (laserScan->ranges.size() / 2) + (laserFOV / 2);
    for (unsigned int i = rangeStart; i <= rangeEnd; i++)
    {
        if (laserScan->ranges.at(i) < laserReading_)
        {
            laserReading_ = laserScan->ranges.at(i);
        }
    }
    return laserReading_;

}

