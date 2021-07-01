#include "uav_com/MasterAPI.h"

MasterAPI::MasterAPI(const std::string &callerId) 
    : m_callerId(callerId)
{ }

MasterAPI::TopicTypesMapPtr MasterAPI::getTopicsTypes() 
{
    XmlRpc::XmlRpcValue params(m_callerId), response, payload;

    if( !ros::master::execute(m_getTopicTypes, params, response, payload, true) )
    {
        return nullptr;
    }

    auto result = std::make_shared< std::map<std::string, std::string> >();
    for(int i=0; i < payload.size(); ++i)
    {
        auto topic = std::string(payload[i][0]);
        auto type = std::string(payload[i][1]);
        result->emplace(topic, type);
    }

    return result;
}

MasterAPI::PubSubListPtr MasterAPI::getPublishedSubscribedTopics() 
{
    XmlRpc::XmlRpcValue params(m_callerId), response, payload;

    if( !ros::master::execute(m_getSystemState, params, response, payload, true) )
    {
        return nullptr;
    }
   
    auto result = std::make_shared<std::vector<std::vector<std::string>>>(); 

    result->emplace_back(std::vector<std::string>()); //published topics vector creation
    auto& publishedTopicsResult = (*result)[0]; //copy to
    auto& publishedTopics = payload[0]; //copy from

    for(int topicIndex=0; topicIndex < publishedTopics.size(); ++topicIndex)
    {
        // std::cout << publishedTopics[topicIndex][0] << " " << publishedTopics[topicIndex][1] << std::endl;
        publishedTopicsResult.emplace_back(publishedTopics[topicIndex][0]);
    }

    result->emplace_back(std::vector<std::string>()); //subscribed topics vector creation
    auto& subscribedTopicsResult = (*result)[1]; //copy to
    auto& subscribedTopics = payload[1]; //copy from

    // std::cout << "subscribed" << std::endl;
    for(int topicIndex=0; topicIndex < subscribedTopics.size(); ++topicIndex)
    {
        // std::cout << subscribedTopics[topicIndex][0] << " " << subscribedTopics[topicIndex][1] << std::endl;
        subscribedTopicsResult.emplace_back(subscribedTopics[topicIndex][0]);
    }

    return result;
}