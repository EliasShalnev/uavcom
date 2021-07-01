#include <ros/ros.h>

#include "com_sim/ComSimMonitor.h"


const std::string &nodename = "com_sim";

int main(int argc, char** argv)
{
    ros::init(argc, argv, nodename);

    auto comSimMonitor = ComSimMonitor::getInstance();

    ros::spin();

    return 0;
}