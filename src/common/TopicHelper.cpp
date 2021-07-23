#include "common/TopicHelper.h"

#include <ros_msg_parser/ros_parser.hpp>


TopicHelper::TopicHelper(const def::TopicName& topicName) 
    : m_nsParser(topicName)
{ }

TopicHelper::TopicHelper(const TopicHelper& rhs) 
    : m_nsParser(rhs.m_nsParser)
{ }


bool TopicHelper::isUavcomTopic()
{
    return m_nsParser[NS_INDEX]+m_nsParser[NODE_NAME_INDEX] == ros::this_node::getName();
}


def::TopicName TopicHelper::getRemoteTopicName() 
{
    if( isUavcomTopic() )
    {
        return m_nsParser.getSubNameSpace(NODE_NAME_INDEX+1, m_nsParser.size()-1);
    }
    else { return def::TopicName(); }
}


def::TopicName TopicHelper::deleteFirstSegment() 
{
    def::TopicName result;
    for(auto segment = m_nsParser.begin()+1; segment < m_nsParser.end(); ++segment)
    {
        result.append(*segment);
    }
    return result;
}
