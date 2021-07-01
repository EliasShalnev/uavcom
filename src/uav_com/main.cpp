#include <ros/ros.h>

#include "uav_com/UavComMonitor.h"

#include <vector>

const std::string &nodeName = "uav_com";

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    auto topicMonitor = UavComMonitor::getInstance();

    ros::spin();
    
    return 0;
}