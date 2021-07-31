#pragma once

#include "com_sim/ComSim.h"

class SlaveComSim : public ComSim
{
public:
    SlaveComSim(const def::BoardName& boardName,
                ComSimObserver& comSimObserver);
    SlaveComSim(const SlaveComSim&) = delete;
    SlaveComSim& operator=(const SlaveComSim&) = delete;
    ~SlaveComSim() = default;

private:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

protected:
    bool check(const ComSim::Ptr from) const override;
};

