#pragma once

#include "ros_msg_parser/ros_parser.hpp"

inline bool isUavcomTopic(const std::string &topicName)
{
    return topicName.rfind(ros::this_node::getName(), 0) != std::string::npos;
}


inline std::string getRemoteTopicName(const std::string &topicName)
{
    std::string result;
    if ( isUavcomTopic(topicName) )
    {
        uint32_t prefixLength = ros::this_node::getName().size();
        result = topicName.substr( prefixLength, topicName.size() );
    }
    return result;
}


inline std::string getFirstSerment(const std::string& topicName) 
{
    auto pos = topicName.find('/', 1); //find second slash
    return topicName.substr(0, pos);
}

//Getting bytearray from ShapeShifter message
inline void getByteArray(const RosMsgParser::ShapeShifter& msg,  
                         std::vector<uint8_t>& byteArray)
{
    byteArray.resize( msg.size() );

    ros::serialization::IStream iStream(byteArray.data(), byteArray.size());
    msg.write(iStream);
}
