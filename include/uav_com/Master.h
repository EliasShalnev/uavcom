#pragma once

#include "uav_com/Slave.h"


class Master : public Slave
{
public:
    Master(const def::BoardName& boardName);
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;
    virtual ~Master() = default;

protected:
    OutputUavStream* getReachableOutput(const def::BoardName& destination) override;
    bool isTopicStreamed(const def::TopicName& topicName) override;

protected:
    InputUavStream  m_coneInput;
    OutputUavStream m_coneOutput;
};

