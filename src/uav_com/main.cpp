#include <ros/ros.h>

#include "common/common.h"

#include "uavcom/Heartbeat.h"

#include "uav_com/UavComMonitor.h"


const std::string &nodeName = "uav_com";

ros::Publisher* heartbeatPub;
void publishHeartBeat(const ros::TimerEvent& event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle;

    const std::string heartBeatTopicName = ros::this_node::getName() + def::BROADCAST + 
                                           ros::this_node::getNamespace() + def::HEARTBEAT;
    heartbeatPub = new ros::Publisher( nodeHandle.advertise<uavcom::Heartbeat>(heartBeatTopicName, 10) );

    ros::Timer heartbeatTimer( nodeHandle.createTimer(ros::Duration(0.2), publishHeartBeat) );

    auto topicMonitor = def::UavComMonitor::getInstance();

    ros::spin();
    
    return 0;
}


void publishHeartBeat(const ros::TimerEvent& event) 
{
    uavcom::Heartbeat heartbeat;
    heartbeat.from = ros::this_node::getNamespace();

    heartbeatPub->publish(heartbeat);
}
