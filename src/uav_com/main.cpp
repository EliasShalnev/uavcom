#include <ros/node_handle.h>
#include <ros/master.h>

#include "common/globals.h"

#include "uavcom/Heartbeat.h"

#include "common/TopicMonitor.h"
#include "uav_com/TopicFilter.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, def::g_uavNodeName);
    ros::NodeHandle nodeHandle("~");

    def::BoardName boardName = ros::this_node::getNamespace();
    if( boardName.find(UavCom::MASTER) != std::string::npos && 
        boardName.find(UavCom::SLAVE) != std::string::npos ) 
    {
        ROS_ERROR_STREAM("Unknown board name: " << boardName 
                         << ". It should be " << UavCom::MASTER << " or " << UavCom::SLAVE);
        return -1;
    }    
    
    const std::string heartBeatTopicName = def::g_broadcast + ros::this_node::getNamespace() + '/' + def::g_heartbeat;
    ros::Publisher heartbeatPub = nodeHandle.advertise<uavcom::Heartbeat>(heartBeatTopicName, 10);

    ros::Timer heartbeatTimer( nodeHandle.createTimer(ros::Duration(0.2), 
                                                      [&heartbeatPub](const ros::TimerEvent& event)
    {
        uavcom::Heartbeat heartbeat;
        heartbeat.from = ros::this_node::getNamespace();

        heartbeatPub.publish(heartbeat);
    }) );

    TopicFilter topicFilter(boardName);
    auto checkPubTopics = std::bind(&TopicFilter::checkPublishedTopics, &topicFilter, std::placeholders::_1);
    auto checkSubTopics = std::bind(&TopicFilter::checkSubscribedTopics, &topicFilter, std::placeholders::_1);

    TopicMonitor topicMonitor;
    topicMonitor.onPublishedTopics(checkPubTopics);
    topicMonitor.onSubscribedTopics(checkSubTopics);

    ros::spin();
    
    return 0;
}

