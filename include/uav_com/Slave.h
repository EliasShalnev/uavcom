#pragma once

#include "uavcom/StreamTopic.h"

#include "uav_com/UavCom.h"


class Slave : public UavCom
{
public:
    Slave(const def::BoardName& boardName);
    Slave(const Slave&) = delete;
    Slave& operator=(const Slave&) = delete;
    virtual ~Slave() = default;

protected:
    OutputUavStream* getReachableOutput(const def::BoardName& destination) override;
    bool isTopicStreamed(const def::TopicName& topicName) override;

protected:
    InputUavStream  m_input;
    OutputUavStream m_output;
};
