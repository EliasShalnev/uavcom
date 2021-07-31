#include "uav_com/OutputUavStream.h"

#include "common/TopicHelper.h"

#include "uavcom/Heartbeat.h"


OutputUavStream::OutputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName)
    : m_nodeHandle(nodeHandle)
    , m_output( nodeHandle.advertise<uavcom::UavMessage>(streamName, 10) )
    , m_subCheckTimer( nodeHandle.createTimer(m_checkTimerFreq, 
                                              &OutputUavStream::checkSubscribers,
                                              this) )
{ 
    const std::string heartBeatTopicName = ros::this_node::getName() + '/' + def::g_broadcast + 
                                           ros::this_node::getNamespace() + '/' + def::g_heartbeat;
    redirectToOutput(heartBeatTopicName);
}

void OutputUavStream::redirectToOutput(const def::TopicName& topicName) 
{
    //check if published topic have already subscribed by this node
    if( m_toOutputTopics.end() != m_toOutputTopics.find(topicName) ) { return; }

    ROS_INFO_STREAM("Subscribing to topic: " << topicName);

    boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
    callback = [this, topicName](const RosMsgParser::ShapeShifter::ConstPtr &msg)
    {
        //preparing output topic msg
        uavcom::UavMessage::Ptr uavMessage(new uavcom::UavMessage);
        uavMessage->topicName = TopicHelper(topicName).getRemoteTopicName();
        uavMessage->MD5Sum = msg->getMD5Sum();
        uavMessage->dataType = msg->getDataType();
        getByteArray( *msg, uavMessage->byteArray );

        m_output.publish(uavMessage);
    };

    ros::Subscriber subscriber = m_nodeHandle.subscribe(topicName, 10, callback);
    m_toOutputTopics.emplace(topicName, subscriber);
}


bool OutputUavStream::contains(const def::TopicName& topicName) const
{
    return m_toOutputTopics.end() !=  m_toOutputTopics.find(topicName);
}


void OutputUavStream::publish(const uavcom::UavMessage::Ptr& msg) 
{
    m_output.publish(msg);
}


void OutputUavStream::checkSubscribers(const ros::TimerEvent& event)
{
    for( auto it = m_toOutputTopics.begin(); it != m_toOutputTopics.end(); )
    {
        auto topicName = it->first;
        if( TopicHelper(topicName).isUavcomTopic() )
        { 
            if(it->second.getNumPublishers() == 0)
            {
                it = m_toOutputTopics.erase(it);
                ROS_INFO_STREAM("Unsubscribing from: " << topicName);
            }
        }
        ++it;
    }
}
