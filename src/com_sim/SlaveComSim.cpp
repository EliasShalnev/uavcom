#include "com_sim/SlaveComSim.h"


SlaveComSim::SlaveComSim(const def::BoardName& boardName,
                         ComSimObserver& comSimObserver) 
    : ComSim(boardName, ComSim::IOType::Slave, comSimObserver)

{ 
    m_input = m_nh.advertise<uavcom::UavMessage>(def::g_input, 10);

    m_output = m_nh.subscribe<uavcom::UavMessage>(def::g_output, 10,
                                                  &SlaveComSim::ouputHandle, this);
}


bool SlaveComSim::check(const ComSim* from) 
{
    if(from->getIOType() == ComSim::Slave) { return false; }
    else if(from->getIOType() == ComSim::Master) { return isSlaveInCone(from, this); }

    return false;
}

//TODO - убрать костыль
void SlaveComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    ComSim::ouputHandle(uavMessage);
}
