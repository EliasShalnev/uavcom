#pragma once

#include "com_sim/ComSim.h"

class MasterComSim : public ComSim
{
public:
    MasterComSim(const def::BoardName& boardName,
                 ComSimObserver& comSimObserver);
    MasterComSim(const MasterComSim&) = delete;
    MasterComSim& operator=(const MasterComSim&) = delete;
    ~MasterComSim() = default;

private: 
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

protected:
    bool check(const ComSim::Ptr from) override;
};