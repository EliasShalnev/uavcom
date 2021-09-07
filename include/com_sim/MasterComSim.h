#pragma once

#include "com_sim/ComSim.h"

class MasterComSim : public ComSim
{
public:
    MasterComSim(Model::ModelName modelName,
                 const def::BoardName& boardName,
                 ComSimObserver& comSimObserver);
    MasterComSim(const MasterComSim&) = delete;
    MasterComSim& operator=(const MasterComSim&) = delete;
    ~MasterComSim() = default;

    IOType getIOType() const override { return IOType::Master; }

    IOName getIOName() const override { return m_boardName+'/'+IOTypeToStr(IOType::Master); }

private: 
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

protected:
    bool check(const ComSim::Ptr from) const override;
};