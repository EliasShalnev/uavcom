#include "uav_com/Slave.h"

#include "uavcom/StreamTopic.h"

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

    OutputUavStream* outputUavStream = getReachableOutput(destination);

    if( nullptr != outputUavStream ) { outputUavStream->redirectToOutput(topicName); }
    else { ROS_INFO_STREAM("Destination " << destination << " is unreachable."); }
}


void Slave::streamTopicRequest(const TopicName& topicName) 
{
    m_streamTopicClient.streamTopicRequest(topicName);
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
    , m_server( enclose->m_nodeHandle.advertiseService(enclose->STREAM_TOPIC_SRV_NAME,
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
    //TODO create auto delete when unsubscribe
    
    //check if subscribed topic have already published by this node
    if( m_enclose->m_input.contains(topicName) ) { return; }

    std::string remoteTopicName = getRemoteTopicName(topicName);
    std::string destination = ros::this_node::getNamespace();

    uavcom::StreamTopic streamTopic;
    streamTopic.request.topicName = remoteTopicName;
    streamTopic.request.destination = destination;

    const std::string serviceName = getFirstSegment(remoteTopicName) + '/' + m_enclose->STREAM_TOPIC_SRV_NAME;

    ros::ServiceClient streamTopicCLient = m_enclose->m_nodeHandle.serviceClient<uavcom::StreamTopic>(serviceName);

    ROS_INFO_STREAM("Calling " << serviceName << " service.");
    if( streamTopicCLient.call(streamTopic) )
    {
        const auto& status = streamTopic.response.status;

        //TODO maybe create publisher to stop spam with request 

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




} //namespace def