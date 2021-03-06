#include <iostream>

#include <ros/ros.h>

#include <std_msgs/String.h>

const std::string &nodeName = "scoutToBombers";

void callback1(const std_msgs::String::ConstPtr& msg);

void callback2(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    ros::NodeHandle nodeHandle;

    ros::Subscriber sub1 = nodeHandle.subscribe("/bomber1/uav_com/scout0/topic1", 10, 
                                                callback1);

    ros::Subscriber sub2 = nodeHandle.subscribe("/bomber2/uav_com/scout0/topic1", 10,
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
