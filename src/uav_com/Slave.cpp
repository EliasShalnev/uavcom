#include "uav_com/Slave.h"

#include "uavcom/StreamTopic.h"

#include "common/TopicHelper.h"

#include "uav_com/ReasonCodes.h"


Slave::Slave(const def::BoardName& boardName) 
    : UavCom(boardName)
    , m_slaveInput( new InputUavStream(m_nh, def::g_slaveIOPrefix+def::g_input) )
    , m_slaveOutput( new OutputUavStream(m_nh, def::g_slaveIOPrefix+def::g_output) )
{ }


OutputUavStream::Ptr Slave::getReachableOutput(const def::BoardName& destination)
{
    if( m_slaveInput->isReachable(destination) ) { return m_slaveOutput; }
    return nullptr;
}


bool Slave::isTopicStreamed(const def::TopicName& topicName) 
{
    return m_slaveInput->contains(topicName);
}

