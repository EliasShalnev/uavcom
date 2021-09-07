#include "uav_com/Slave.h"

#include "uavcom/StreamTopic.h"

#include "common/TopicHelper.h"

#include "uav_com/ReasonCodes.h"


Slave::Slave(const def::BoardName& boardName) 
    : UavCom(boardName)
    , m_input(m_nh, def::g_input)
    , m_output(m_nh, def::g_output)
{ }


OutputUavStream* Slave::getReachableOutput(const def::BoardName& destination)
{
    if( m_input.isReachable(destination) ) { return &m_output; } //FIXME: potential error. Bad interface. Probably return weak_ptr or smth
    return nullptr;
}


bool Slave::isTopicStreamed(const def::TopicName& topicName) 
{
    return m_input.contains(topicName);
}

