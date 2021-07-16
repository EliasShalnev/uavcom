#pragma once

#include <unordered_map>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "common/common.h"

#include "uavcom/UavMessage.h"


class ComSim;

class ComSimObserver
{
public:
    ComSimObserver() = default;
    ComSimObserver(const ComSimObserver& orig) = delete;
    ComSimObserver& operator=(const ComSimObserver& orig) = delete;
    ~ComSimObserver();

    void publishToInput(const def::BoardName& from, 
                        const uavcom::UavMessage::ConstPtr& uavMessage);

    void checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics);

private:
    std::unordered_map<def::BoardName, ComSim*> m_store;
};

