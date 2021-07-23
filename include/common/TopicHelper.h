#pragma once

#include "common/globals.h"

#include "common/NSParser.h"

class TopicHelper
{

public:
    TopicHelper(const def::TopicName& topicName);
    TopicHelper(const TopicHelper& rhs);
    TopicHelper& operator=(const TopicHelper&) = delete;
    ~TopicHelper() = default;


    inline std::size_t size() { return m_nsParser.size(); }

    NSParser::Segment& operator[](std::size_t n) { return m_nsParser[n]; }

    auto begin() { return m_nsParser.begin(); }
    auto end() { return m_nsParser.end(); }

    bool isUavcomTopic();

    def::TopicName getRemoteTopicName();

    def::TopicName deleteFirstSegment();

private:
    NSParser m_nsParser;

    const std::size_t NS_INDEX = 0;
    const std::size_t NODE_NAME_INDEX = 1;

};

