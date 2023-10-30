
#include "ros/ros.h"
#include "followTarget.h"

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Now running followTarget.cpp!");
   // ROS Initialise
    ros::init(argc, argv, "followTarget");
    ros::NodeHandle nh;
    // Create Thread
    std::shared_ptr<FollowTarget> FollowTargetPtr(new FollowTarget(nh));
    std::thread t(&FollowTarget::run, FollowTargetPtr);

    // ros::spin() will enter a loop, use callbacks
    ros::spin();

    // Shut down and close the thread
    ros::shutdown();

    t.join();

    return 0;
}
