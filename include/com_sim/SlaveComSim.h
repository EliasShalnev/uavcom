#pragma once

#include "com_sim/ComSim.h"

class SlaveComSim : public ComSim
{
public:
    SlaveComSim(Model::ModelName modelName,
                const def::BoardName& boardName,
                ComSimObserver& comSimObserver);
    SlaveComSim(const SlaveComSim&) = delete;
    SlaveComSim& operator=(const SlaveComSim&) = delete;
    ~SlaveComSim() = default;

    IOType getIOType() const { return IOType::Slave; }
    
    IOName getIOName() const override { return m_boardName + '/' + IOTypeToStr(IOType::Slave); }

private:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

protected:
    bool check(const ComSim::Ptr from) const override;
};

