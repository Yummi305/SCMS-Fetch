
#include "ros/ros.h"
#include <thread>
#include "../include/followTarget.h"

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Now running followTarget.cpp!");
    /* The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "followTarget");


    ros::NodeHandle nh;


    // FollowTarget followTarget;

    /**
     * Let's start seperate thread first, to do that we need to create object
     * and thereafter start the thread on the function desired
     */
    std::shared_ptr<FollowTarget> FollowTargetPtr(new followTarget(nh));
        
    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    ros::spin();

    /**
     * Let's cleanup everything, shutdown ros and join the thread
     */
    ros::shutdown();

    return 0;
}