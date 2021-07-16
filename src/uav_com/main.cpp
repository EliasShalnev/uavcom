#include <ros/ros.h>

#include "common/common.h"

#include "uavcom/Heartbeat.h"

#include "uav_com/UavComMonitor.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, def::g_uavNodeName);
    ros::NodeHandle nodeHandle;
    
    const std::string heartBeatTopicName = ros::this_node::getName() + def::g_broadcast + 
                                           ros::this_node::getNamespace() + def::g_heartbeat;
    ros::Publisher heartbeatPub = nodeHandle.advertise<uavcom::Heartbeat>(heartBeatTopicName, 10);

    ros::Timer heartbeatTimer( nodeHandle.createTimer(ros::Duration(0.2), 
                                                      [&heartbeatPub](const ros::TimerEvent& event)
    {
        uavcom::Heartbeat heartbeat;
        heartbeat.from = ros::this_node::getNamespace();

        heartbeatPub.publish(heartbeat);
    }) );

    auto topicMonitor = def::UavComMonitor::getInstance();

    ros::spin();
    
    return 0;
}

