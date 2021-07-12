#pragma once

#include <ros/ros.h>

#include "uavcom/StreamTopic.h"

namespace def
{

class StreamTopicServer
{
public:
    StreamTopicServer(ros::NodeHandle nodeHandle, ros::Publisher output);
    ~StreamTopicServer() = default;

private:
    inline bool isDestExist(const std::string& topicName, 
                            const std::string& dest) const;
    bool onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                              uavcom::StreamTopic::Response& res);
private:
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_output; //"output" topic Publisher

    ros::ServiceServer m_server; //starting streaming requested topic to "output" topic

    std::unordered_map<std::string, ros::Subscriber> m_toOutputTopics; //topics directed to "output" topic

    std::unordered_multimap<std::string, std::string> m_destinations;
};

} //namespace def