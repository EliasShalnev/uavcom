#include <iostream>

#include <ros/ros.h>

#include <std_msgs/String.h>

const std::string &nodeName = "bombersToScout";

void callback1(const std_msgs::String::ConstPtr& msg);

void callback2(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    ros::NodeHandle nodeHandle;

    ros::Subscriber sub1 = nodeHandle.subscribe("/scout0/uav_com/bomber1/topic2", 10, 
                                                callback1);

    ros::Subscriber sub2 = nodeHandle.subscribe("/scout0/uav_com/bomber2/topic2", 10,
                                                callback2);

    ros::spin();
    
    return 0;
}


void callback1(const std_msgs::String::ConstPtr& msg) 
{
    ROS_INFO_STREAM("callback 1 " << *msg);
}


void callback2(const std_msgs::String::ConstPtr& msg) 
{
    ROS_INFO_STREAM("callback 2 " << *msg);
}
