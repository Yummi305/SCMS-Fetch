#ifndef LASERSCAN.H
#define LASERSCAN.H

#include "sensor_msgs/LaserScan.h"


/*Class declerations here for laserScan.cpp*/
class laserScan{
    public:
        LaserDetection();

        bool ObjectDetect(sensor_msgs::LaserScan::ConstPtr laserScan);
        double LaserReading(sensor_msgs::LaserScan::ConstPtr laserScan);

    private:
        double laserReading;

};

#endif