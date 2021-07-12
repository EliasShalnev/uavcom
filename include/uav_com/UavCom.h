#pragma once

#include "common/common.h"

#include "uav_com/OutputUavStream.h"

namespace def {

class UavCom
{
public:
    UavCom() = default;
    UavCom(const UavCom& origin) = delete;
    UavCom& operator=(const UavCom& origin) = delete;
    virtual ~UavCom() = default;

    virtual void redirectToOutput(const TopicName& topicName) = 0;

protected:
    // virtual bool containsInOutput(const TopicName& topicName) = 0;
    virtual OutputUavStream* getReachableOutput(const Destination& destination) = 0;
};

} //namespace def
