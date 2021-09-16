#include "uav_com/InputUavStream.h"

#include "uavcom/StreamTopic.h"

#include "common/TopicHelper.h"

#include "uav_com/ReasonCodes.h"


InputUavStream::InputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName)
    : m_nodeHandle(nodeHandle)
    , m_input( nodeHandle.subscribe<uavcom::UavMessage>(streamName, 10, 
                                                        &InputUavStream::inputHandle, this) )
{ }


bool InputUavStream::contains(const def::TopicName& topicName) const
{
    return m_fromInputTopics.end() !=  m_fromInputTopics.find(topicName);
}


bool InputUavStream::isReachable(const def::BoardName& boardName) const
{
    def::TopicName heartbeatTopic = ros::this_node::getNamespace() + boardName + '/' + def::g_heartbeat;
    
    if( m_fromInputTopics.find(heartbeatTopic) != m_fromInputTopics.end() ) { return true; }
    return false;
}


void InputUavStream::inputHandle(const uavcom::UavMessage::ConstPtr& uavMsg) 
{
    def::TopicName topicName = uavMsg->topicName;
    TopicHelper topicHelper(topicName);

    const std::string ns = *topicHelper.begin();
    //filter not for this node msgs
    if( ns != ros::this_node::getNamespace() ) 
    { 
        if(ns != '/'+def::g_broadcast) { return; } //if it isn't broadcast msg 
        auto newTopicName = topicHelper.deleteFirstSegment();
        topicName = ros::this_node::getNamespace() + newTopicName;
    }
    RosMsgParser::ShapeShifter msg;
    msg.morph(
        uavMsg->MD5Sum,
        uavMsg->dataType,
        ""
    );
    std::vector<uint8_t> byteArray = uavMsg->byteArray; //без копирования не получится так как uavMsg является const
    setByteArray(msg, byteArray);

    auto pubIt = m_fromInputTopics.find(topicName);
    if(m_fromInputTopics.end() == pubIt)
    {
        ROS_INFO_STREAM("Advertising to topic: " << topicName);
        ros::Timer newTimer = m_nodeHandle.createTimer(ros::Duration(5),
                                                       [this, topicName](const ros::TimerEvent& event)
        {
            ROS_INFO_STREAM("Unadvertising topic: " << topicName);
            m_fromInputTopics.erase(topicName);
        });
        
        ros::Publisher newPub = msg.advertise(m_nodeHandle, topicName, 10);
        m_fromInputTopics.insert( {topicName, {newTimer, newPub} } );
        newPub.publish( msg );
    } else 
    { 
        auto pubTimer = pubIt->second.first;
        auto expiringPub = pubIt->second.second;

        pubTimer.stop();
        expiringPub.publish( msg ); 
        pubTimer.start();
    }  
}
