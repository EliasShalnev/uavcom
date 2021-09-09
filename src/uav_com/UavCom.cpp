#include "uav_com/UavCom.h"


#include "common/TopicHelper.h"


const std::string UavCom::MASTER = "scout";
const std::string UavCom::SLAVE  = "bomber";


UavCom::UavCom(const def::BoardName& boardName) 
    : m_boardName(boardName)
    , m_nh("~")
    , m_streamTopicServer(this)
    , m_streamTopicClient(this)
{ }


void UavCom::redirectToOutput(const def::TopicName& topicName) 
{
    TopicHelper topicHelper(topicName);

    auto remoteTopic = topicHelper.getRemoteTopicName();
    auto destination = *TopicHelper(remoteTopic).begin();

    auto outputUavStream = getReachableOutput(destination);

    if( nullptr != outputUavStream ) { outputUavStream->redirectToOutput(topicName); }
    else { ROS_WARN_STREAM("Destination " << destination << " is unreachable."); }
}


void UavCom::streamTopicRequest(const def::TopicName& topicName) 
{
    m_streamTopicClient.streamTopicRequest(topicName);
}

/****Server****/
#include "uav_com/ReasonCodes.h"

UavCom::StreamTopicServer::StreamTopicServer(UavCom* enclose) 
    : m_enclose(enclose)
    , m_server( enclose->m_nh.advertiseService(enclose->STREAM_TOPIC_SRV_NAME,
                                               &UavCom::StreamTopicServer::onStreamTopicRequest,
                                               this) )
{}


bool UavCom::StreamTopicServer::onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                                     uavcom::StreamTopic::Response& res) 
{
    const def::TopicName topicName = req.topicName;
    const def::BoardName destination = req.destination;
    
    auto outpuUavStream = m_enclose->getReachableOutput(destination);

    //check if publishers exists
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
                    uavMessage->topicName = destination + '/' + def::g_uavNodeName + topicName;
                    uavMessage->MD5Sum = msg->getMD5Sum();
                    uavMessage->dataType = msg->getDataType();
                    outpuUavStream->getByteArray( *msg, uavMessage->byteArray );

                    outpuUavStream->publish(uavMessage);
                }
            };
            ros::Subscriber subscriber = m_enclose->m_nh.subscribe(topicName, 10, callback);
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


inline bool UavCom::StreamTopicServer::isDestExist(const def::TopicName& topicName, 
                                                   const def::BoardName& dest) const
{
    auto [beginDest, endDest] = m_destinations.equal_range(topicName);
    for(auto destIt = beginDest; destIt != endDest; ++destIt)
    {
        if(destIt->second == dest) { return true; }
    }
    return false;
}

/****Client****/
UavCom::StreamTopicClient::StreamTopicClient(UavCom* enclose) 
    : m_enclose(enclose)
{ }


void UavCom::StreamTopicClient::streamTopicRequest(const def::TopicName& topicName) 
{
    //TODO create auto delete when unsubscribe
    
    //check if subscribed topic have already published by this node
    if( m_enclose->isTopicStreamed(topicName) ) { return; }

    def::TopicName remoteTopicName = TopicHelper(topicName).getRemoteTopicName();
    def::BoardName destination = ros::this_node::getNamespace();

    uavcom::StreamTopic streamTopic;
    streamTopic.request.topicName = remoteTopicName;
    streamTopic.request.destination = destination;

    const std::string serviceName = *TopicHelper(remoteTopicName).begin() + "/uav_com/"
                                    + m_enclose->STREAM_TOPIC_SRV_NAME;

    ros::ServiceClient streamTopicCLient = m_enclose->m_nh.serviceClient<uavcom::StreamTopic>(serviceName);

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


