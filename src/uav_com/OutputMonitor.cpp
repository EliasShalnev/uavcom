#include "uav_com/OutputMonitor.h"

#include "uavcom/UavMessage.h"

#include "common/TopicNameHelper.h"


OutputMonitor::OutputMonitor(ros::NodeHandle nodeHandle)
    : m_nodeHandle(nodeHandle)
    , m_output( nodeHandle.advertise<uavcom::UavMessage>(ros::this_node::getName()+"/output", 10) )
    , m_streamTopicServer(nodeHandle, m_output)
    , m_subExpireTimer( nodeHandle.createTimer(ros::Duration(5), 
                                               &OutputMonitor::checkSubscribers,
                                               this) )
{ }

void OutputMonitor::redirectToOutput(const std::string& topicName) 
{
    //check if published topic have already subscribed by this node
    if( m_toOutputTopics.end() != m_toOutputTopics.find(topicName) ) { return; }

    ROS_INFO_STREAM("Subscribing to topic: " << topicName);

    boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
    callback = [this, topicName](const RosMsgParser::ShapeShifter::ConstPtr &msg)
    {
        //preparing output topic msg
        uavcom::UavMessage uavMessage;
        uavMessage.topicName = getRemoteTopicName(topicName);
        uavMessage.MD5Sum = msg->getMD5Sum();
        uavMessage.dataType = msg->getDataType();
        getByteArray( *msg, uavMessage.byteArray );

        m_output.publish(uavMessage);
    };

    ros::Subscriber subscriber = m_nodeHandle.subscribe(topicName, 10, callback);
    m_toOutputTopics.emplace(topicName, subscriber);
}


void OutputMonitor::checkSubscribers(const ros::TimerEvent& event) 
{
    for( auto it = m_toOutputTopics.begin(); it != m_toOutputTopics.end(); )
    {
        if( 0 == it->second.getNumPublishers() )
        {
            auto tpName = it->first;

            it = m_toOutputTopics.erase(it);
            ROS_INFO_STREAM("Unsubscribing from: " << tpName);
        }
        else { ++it; }
    }
}
