#pragma once

#include <unordered_map>
#include <string>

#include "uavcom/UavMessage.h"

#include "com_sim/ComSim.h"


class ComSimObserver
{
public:
    ComSimObserver() = default;
    ComSimObserver(const ComSimObserver& orig) = delete;
    ComSimObserver& operator=(const ComSimObserver& orig) = delete;
    ~ComSimObserver();

    void publishToInput(const ComSim::IOName& fromIoName, 
                        const uavcom::UavMessage::ConstPtr& uavMessage);

    void checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics);

private:
    std::unordered_map<ComSim::IOName, ComSim*> m_store; //TODO - заменить на shared_ptr
};

