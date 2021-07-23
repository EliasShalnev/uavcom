#include "uav_com/TopicFilter.h"

#include "common/TopicHelper.h"

#include "uav_com/Slave.h"
#include "uav_com/Master.h"


TopicFilter::TopicFilter(const def::BoardName& boardName) 
{
    if( boardName.find(UavCom::MASTER) != std::string::npos )
    {
        m_uavCom.reset(new Master(boardName));
    }
    else if(boardName.find(UavCom::SLAVE) != std::string::npos)
    {
        m_uavCom.reset(new Slave(boardName));
    }
    else { ROS_ERROR_STREAM("Unknown board name: " << boardName 
                            << ". It should be " << UavCom::MASTER << " or " << UavCom::SLAVE); }

}


void TopicFilter::checkPublishedTopics(const XmlRpc::XmlRpcValue& pubTopics) 
{
    forEachUavComTopic(pubTopics, [this](const def::TopicName& topicName)
    {
        m_uavCom->redirectToOutput(topicName);
    });
}


void TopicFilter::checkSubscribedTopics(const XmlRpc::XmlRpcValue& subTopics) 
{
    forEachUavComTopic(subTopics, [this](const def::TopicName& topicName)
    {
        m_uavCom->streamTopicRequest(topicName);
    });
}


void TopicFilter::forEachUavComTopic(const XmlRpc::XmlRpcValue &topics,
                                     const std::function<void(const def::TopicName&)>& UnaryFunc)
{
    for(int topicIndex=0; topicIndex < topics.size(); ++topicIndex)
    {
        const def::TopicName topicName = std::string( topics[topicIndex][0] );

        TopicHelper topicHelper(topicName);

        //check if node name prefix is presented in the name of the topic
        if(topicHelper.size() < MIN_TOPIC_SIZE) { continue; }
        if( !topicHelper.isUavcomTopic() ) { continue; }
        //check if user is published 
        if( !isUserNodeExists( topics[topicIndex][1] ) ) { continue; }

        UnaryFunc(topicName);
    }
}


inline bool TopicFilter::isUserNodeExists(const XmlRpc::XmlRpcValue &nodeNames) const
{
    for(int i=0; i < nodeNames.size(); ++i)
    {
        if( std::string(nodeNames[i]) != ros::this_node::getName() ) { return true; }  
    }
    return false;
}
