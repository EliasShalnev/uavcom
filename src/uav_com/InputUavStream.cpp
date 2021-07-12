#include "uav_com/InputUavStream.h"

#include "uavcom/StreamTopic.h"

#include "uav_com/ReasonCodes.h"


namespace def
{


InputUavStream::InputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName)
    : m_nodeHandle(nodeHandle)
    , m_input( nodeHandle.subscribe<uavcom::UavMessage>( ros::this_node::getName()+'/'+streamName, 10, 
                                                         &InputUavStream::inputHandle, this) )
{ }

// void InputUavStream::streamTopicRequest(const std::string& topicName) 
// {
//     //TODO create auto delete when unsubscribe
    
//     //check if subscribed topic have already published by this node
//     if( m_fromInputTopics.end() != m_fromInputTopics.find(topicName) ) { return; }

//     std::string remoteTopicName = getRemoteTopicName(topicName);
//     std::string destination = getFirstSegment(topicName);

//     uavcom::StreamTopic streamTopic;
//     streamTopic.request.topicName = remoteTopicName;
//     streamTopic.request.destination = destination;

//     std::string serviceName = getFirstSegment(remoteTopicName) + "/stream_topic_service";
  
//     ros::ServiceClient streamTopicCLient = m_nodeHandle.serviceClient<uavcom::StreamTopic>(serviceName);

//     if( streamTopicCLient.call(streamTopic) )
//     { 
//         const auto& status = streamTopic.response.status;

//         //TODO create publisher to stop spam with request 

//         //Only for log
//         if(status == StreamTopic::OK) {
//             ROS_INFO_STREAM("Stream topic request [topicName=" << remoteTopicName <<
//                             ", destination=" << destination << "] was accepted.");
//         } else {
//             ROS_WARN_STREAM("Stream topic request [topicName=" << remoteTopicName << 
//                             ", destination=" << destination << "] was denied. "
//                             "[Reason] " << StreamTopic::codeExplanation(status));
//         }
//     } else {
//         ROS_WARN_STREAM("Stream topic request [topicName=" << remoteTopicName << 
//                         ", destination=" << destination << "] was denied.");
//     }
// }


bool InputUavStream::isReachable(const std::string& boardName) 
{
    std::string heartbeatTopic = ros::this_node::getNamespace() + boardName + HEARTBEAT;

    if( m_fromInputTopics.find(heartbeatTopic) != m_fromInputTopics.end() ) { return true; }
    return false;
}


void InputUavStream::inputHandle(const uavcom::UavMessage::ConstPtr& uavMsg) 
{
    std::string topicName = uavMsg->topicName;

    const std::string ns = getFirstSegment(topicName);
    //filter not for this node msgs
    if( ns != ros::this_node::getNamespace() ) 
    { 
        if(ns != BROADCAST) { return; }
        auto newTopicName = deleteFirstSegment(topicName);
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


} //namespace def