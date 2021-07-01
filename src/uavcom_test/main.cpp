#include <iostream>

#include <ros/ros.h>

#include <std_msgs/String.h>

const std::string &nodeName = "uavcom_test";

void callback1(const std_msgs::String::ConstPtr& msg);

void callback2(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    ros::NodeHandle nodeHandle;

    ros::Subscriber sub1 = nodeHandle.subscribe("/uav1/uav_com/uav3/topic1", 10, 
                                                callback1);

    ros::Subscriber sub2 = nodeHandle.subscribe("/uav2/uav_com/uav3/topic1", 10,
                                                callback2);

    ros::spin();
    
    return 0;
}


void callback1(const std_msgs::String::ConstPtr& msg) 
{
    std::cout << "callback 1 " << *msg << std::endl;
}


void callback2(const std_msgs::String::ConstPtr& msg) 
{
    std::cout << "callback 2 " << *msg << std::endl;
}
