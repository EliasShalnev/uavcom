#include "uav_com/InputMonitor.h"

#include "uavcom/StreamTopic.h"

#include "common/common.h"

#include "uav_com/ReasonCodes.h"

InputMonitor::InputMonitor(ros::NodeHandle nodeHandle)
    : m_nodeHandle(nodeHandle)
    , m_input( nodeHandle.subscribe<uavcom::UavMessage>( ros::this_node::getName()+"/input", 10, 
                                                         &InputMonitor::inputHandle, this) )
{ }

void InputMonitor::streamTopicRequest(const std::string& topicName) 
{
    //TODO create auto delete when unsubscribe
    
    //check if subscribed topic have already published by this node
    if( m_fromInputTopics.end() != m_fromInputTopics.find(topicName) ) { return; }

    std::string remoteTopicName = getRemoteTopicName(topicName);
    std::string destination = getFirstSerment(topicName);

    uavcom::StreamTopic streamTopic;
    streamTopic.request.topicName = remoteTopicName;
    streamTopic.request.destination = destination;

    std::string serviceName = getFirstSerment(remoteTopicName) + "/stream_topic_service";
  
    ros::ServiceClient streamTopicCLient = m_nodeHandle.serviceClient<uavcom::StreamTopic>(serviceName);

    if( streamTopicCLient.call(streamTopic) )
    { 
        const auto& status = streamTopic.response.status;

        //TODO create publisher to stop spam with request 

        //Only for log
        if(status == StreamTopic::OK) {
            ROS_INFO_STREAM("Stream topic request [topicName=" << remoteTopicName <<
                            ", destination=" << destination << "] was accepted.");
        } else {
            ROS_WARN_STREAM("Stream topic request [topicName=" << remoteTopicName << 
                            ", destination=" << destination << "] was denied. "
                            "[Reason] " << StreamTopic::codeExplanation(status));
        }
    } else {
        ROS_WARN_STREAM("Stream topic request [topicName=" << remoteTopicName << 
                        ", destination=" << destination << "] was denied.");
    }
}

void InputMonitor::inputHandle(const uavcom::UavMessage::ConstPtr& uavMsg) 
{
    std::string topicName = uavMsg->topicName;

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
        pubTimer.start();
        expiringPub.publish( msg ); 
    }  
}

inline void InputMonitor::setByteArray(RosMsgParser::ShapeShifter& msg,  
                                       std::vector<uint8_t>& byteArray) const
{
    ros::serialization::OStream oStream( byteArray.data(), byteArray.size() );
    msg.read( oStream );  
}
