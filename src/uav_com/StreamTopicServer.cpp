#include "uav_com/StreamTopicServer.h"

#include "ros_msg_parser/ros_parser.hpp"

#include "uavcom/UavMessage.h"
#include "common/common.h"

#include "uav_com/ReasonCodes.h"

namespace def
{

StreamTopicServer::StreamTopicServer(ros::NodeHandle nodeHandle, ros::Publisher output) 
    : m_nodeHandle(nodeHandle)
    , m_output(output)
    , m_server( nodeHandle.advertiseService("stream_topic_service",
                                            &StreamTopicServer::onStreamTopicRequest,
                                            this) )    
{ }


inline bool StreamTopicServer::isDestExist(const std::string& topicName, 
                                           const std::string& dest) const
{
    auto [beginDest, endDest] = m_destinations.equal_range(topicName);
    for(auto destIt = beginDest; destIt != endDest; ++destIt)
    {
        if(destIt->second == dest) { return true; }
    }
    return false;
}


bool StreamTopicServer::onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                             uavcom::StreamTopic::Response& res) 
{
    const std::string topicName = req.topicName;
    const std::string destination = req.destination;

    res.status = StreamTopic::OK;

    if( isDestExist(topicName, destination) ) { res.status = StreamTopic::IsStreaming; }
    else 
    { 
        m_destinations.emplace(topicName, destination); 

        if( m_toOutputTopics.end() == m_toOutputTopics.find(topicName) ) 
        { 
            boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
            callback = [this, topicName](const RosMsgParser::ShapeShifter::ConstPtr &msg)
            {
                auto [beginDest, endDest] = m_destinations.equal_range(topicName);
                for(auto destIt = beginDest; destIt != endDest; ++destIt)
                {
                    std::string destination = destIt->second;
                    uavcom::UavMessage uavMessage;
                    uavMessage.topicName = destination + "/uav_com" + topicName ;
                    uavMessage.MD5Sum = msg->getMD5Sum();
                    uavMessage.dataType = msg->getDataType();
                    getByteArray( *msg, uavMessage.byteArray );

                    m_output.publish(uavMessage);
                }
            };
            ros::Subscriber subscriber = m_nodeHandle.subscribe(topicName, 10, callback);
            m_toOutputTopics.emplace(topicName, subscriber);
        }
    }

    if( StreamTopic::OK == res.status ) {
        ROS_INFO_STREAM("Stream topic request [topicName=" << topicName << 
                        " destination=" << destination << "] is accepted.");
    } else {
        ROS_WARN_STREAM("Stream topic request [topicName=" << topicName <<
                        ", destination=" << destination << "] is denied. " <<
                        "[Reason] " << StreamTopic::codeExplanation(res.status));
    }

    return true;
}

} //namespace def