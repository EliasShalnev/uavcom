#pragma once

#include <memory>

#include <ros/ros.h>

class MasterAPI
{
public:
    using TopicTypesMap = std::map<std::string, std::string>;
    using TopicTypesMapPtr = std::shared_ptr<TopicTypesMap>;
    
    using PubSubList = std::vector<std::vector<std::string>>;
    using PubSubListPtr = std::shared_ptr<PubSubList>;

public:
    MasterAPI(const std::string &callerId);
    MasterAPI(const MasterAPI &origin) = delete;
    MasterAPI& operator=(const MasterAPI &origin) = delete;
    ~MasterAPI() = default;

    TopicTypesMapPtr getTopicsTypes();
    PubSubListPtr getPublishedSubscribedTopics();

private:
    const std::string m_callerId;
    const std::string m_getTopicTypes = "getTopicTypes";   //remote XML_RPC procedure name
    const std::string m_getSystemState = "getSystemState"; //remote XML_RPC procedure name
};
