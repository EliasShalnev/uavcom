#include "uav_com/Slave.h"

#include "uav_com/ReasonCodes.h"

namespace def {

Slave::Slave(ros::NodeHandle nodeHandle) 
    : m_nodeHandle(nodeHandle)
    , m_input(nodeHandle, "input")
    , m_output(nodeHandle, "output")
    , m_streamTopicServer(this)
    , m_streamTopicClient(this)
{ }


void Slave::redirectToOutput(const TopicName& topicName) 
{
    auto remoteTopic = getRemoteTopicName(topicName);
    auto destination = getFirstSegment(remoteTopic);

    if( m_input.isReachable(destination) )
    {
        m_output.redirectToOutput(topicName);
    }
    else { ROS_INFO_STREAM("Destination " << destination << " is unreachable."); }
}


// bool Slave::containsInOutput(const TopicName& topicName) 
// {
    // return m_output.contains(topicName);
// }


OutputUavStream* Slave::getReachableOutput(const Destination& destination)
{
    if( m_input.isReachable(destination) ) { return &m_output; } //FIXME: potential error. Bad interface. Probably return weak_ptr or smth
    return nullptr;
}


Slave::StreamTopicServer::StreamTopicServer(Slave* enclose) 
    : m_enclose(enclose)
    , m_server( enclose->m_nodeHandle.advertiseService("stream_topic_service",
                                                       &Slave::StreamTopicServer::onStreamTopicRequest,
                                                       this) )    
{}


bool Slave::StreamTopicServer::onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                                    uavcom::StreamTopic::Response& res) 
{
    const std::string topicName = req.topicName;
    const std::string destination = req.destination;
    
    OutputUavStream* outpuUavStream = m_enclose->getReachableOutput(destination);

    if(nullptr == outpuUavStream) { res.status = StreamTopic::IsUnreachable; }
    else if( isDestExist(topicName, destination) ) { res.status = StreamTopic::IsStreaming; }
    else
    {
        m_destinations.emplace(topicName, destination); 
        
        if( !outpuUavStream->contains(topicName) )
        {
            boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
            callback = [this, topicName, outpuUavStream](const RosMsgParser::ShapeShifter::ConstPtr &msg)
            {
                auto [beginDest, endDest] = m_destinations.equal_range(topicName);
                for(auto destIt = beginDest; destIt != endDest; ++destIt)
                {
                    std::string destination = destIt->second;
                    uavcom::UavMessage::Ptr uavMessage(new uavcom::UavMessage);
                    uavMessage->topicName = destination + "/uav_com" + topicName;
                    uavMessage->MD5Sum = msg->getMD5Sum();
                    uavMessage->dataType = msg->getDataType();
                    getByteArray( *msg, uavMessage->byteArray );

                    outpuUavStream->publish(uavMessage);
                }
            };
            ros::Subscriber subscriber = m_enclose->m_nodeHandle.subscribe(topicName, 10, callback);
            outpuUavStream->emplace(topicName, subscriber);
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


inline bool Slave::StreamTopicServer::isDestExist(const std::string& topicName, 
                                                  const std::string& dest) const
{
    auto [beginDest, endDest] = m_destinations.equal_range(topicName);
    for(auto destIt = beginDest; destIt != endDest; ++destIt)
    {
        if(destIt->second == dest) { return true; }
    }
    return false;
}


Slave::StreamTopicClient::StreamTopicClient(Slave* enclose) 
    : m_enclose(enclose)
{ }


void Slave::StreamTopicClient::streamTopicRequest(const std::string& topicName) 
{

}




} //namespace def