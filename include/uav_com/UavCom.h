#pragma once

#include <ros/node_handle.h>

#include "common/globals.h"

#include "uav_com/OutputUavStream.h"


class UavCom
{
public:
    const static std::string MASTER;
    const static std::string SLAVE;

public:
    UavCom(const def::BoardName& boardName);
    UavCom(const UavCom&) = delete;
    UavCom& operator=(const UavCom&) = delete;
    virtual ~UavCom() = default;

    virtual void redirectToOutput(const def::TopicName& topicName) = 0;
    virtual void streamTopicRequest(const def::TopicName& topicName) = 0;

protected:
    ros::NodeHandle m_nh;

protected:
    const def::BoardName m_boardName; 
    virtual OutputUavStream* getReachableOutput(const def::BoardName& destination) = 0;
};

