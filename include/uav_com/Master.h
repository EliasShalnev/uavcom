#pragma once

#include "uav_com/Slave.h"

namespace def {


class Master : public Slave
{
public:
    Master(ros::NodeHandle& nodeHandle);
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;
    virtual ~Master() = default;

    void redirectToOutput(const TopicName& topicName) override;
    
    void streamTopicRequest(const TopicName& topicName) override;

protected:
    OutputUavStream* getReachableOutput(const BoardName& destination) override;

protected:
    InputUavStream  m_coneInput;
    OutputUavStream m_coneOutput;
};


} //namespace def