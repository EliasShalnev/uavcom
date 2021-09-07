#include <ros/ros.h>

#include "com_sim/ComSimObserver.h"


const std::string &nodename = "com_sim";


int main(int argc, char** argv)
{
    ros::init(argc, argv, nodename);
    
    ComSimObserver comSimObserver;

    ros::spin();

    return 0;
}