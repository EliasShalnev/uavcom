#include "com_sim/SlaveComSim.h"


SlaveComSim::SlaveComSim(Model::ModelName modelName,
                         const def::BoardName& boardName,
                         ComSimObserver& comSimObserver) 
    : ComSim(modelName, boardName, comSimObserver)
{ 
    m_input = ComSim::m_nh.advertise<uavcom::UavMessage>(def::g_slaveIOPrefix+def::g_input, 10);
    m_output = ComSim::m_nh.subscribe<uavcom::UavMessage>(def::g_slaveIOPrefix+def::g_output, 10,
                                                          &SlaveComSim::ouputHandle, this);
}


void SlaveComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    ComSim::ouputHandle(uavMessage);
}


bool SlaveComSim::check(const ComSim::Ptr from) const 
{
    if(from->getIOType() == ComSim::Slave) { return false; }
    else if(from->getIOType() == ComSim::Master) { return isSlaveInCone(from.get(), this); }

    return false;
}