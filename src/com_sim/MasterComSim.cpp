#include "com_sim/MasterComSim.h"


MasterComSim::MasterComSim(Model::ModelName modelName,
                           const def::BoardName& boardName,
                           ComSimObserver& comSimObserver)
    : ComSim(modelName, boardName, comSimObserver)
{
    m_input = ComSim::m_nh.advertise<uavcom::UavMessage>(def::g_cone+def::g_input, 10);
    m_output = ComSim::m_nh.subscribe<uavcom::UavMessage>(def::g_cone+def::g_output, 10,
                                                          &MasterComSim::ouputHandle, this);
}


void MasterComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    ComSim::ouputHandle(uavMessage);
}


bool MasterComSim::check(const ComSim::Ptr from) const 
{
    if(from->getIOType() == ComSim::Slave) { return isSlaveInCone(this, from.get());  }
    else if(from->getIOType() == ComSim::Master) { return false; }

    return false;
}
