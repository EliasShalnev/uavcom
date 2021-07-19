#include "com_sim/MasterComSim.h"


MasterComSim::MasterComSim(const def::BoardName& boardName,
                           ComSimObserver& comSimObserver)
    : ComSim(boardName, ComSim::IOType::Master, comSimObserver)
{
    m_input = m_nh.advertise<uavcom::UavMessage>(def::g_cone+def::g_input, 10);
    m_output = m_nh.subscribe<uavcom::UavMessage>(def::g_output, 10,
                                                  &MasterComSim::ouputHandle, this);
}

//TODO - убрать костыль
void MasterComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    ComSim::ouputHandle(uavMessage);
}


bool MasterComSim::check(const ComSim* from) 
{
    if(from->getIOType() == ComSim::Slave) { return isSlaveInCone(this, from);  }
    else if(from->getIOType() == ComSim::Master) { return false; }

    return false;
}
