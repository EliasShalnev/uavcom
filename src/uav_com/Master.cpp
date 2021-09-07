#include "uav_com/Master.h"


Master::Master(const def::BoardName& boardName) 
    : Slave(boardName)
    , m_coneInput(m_nh, def::g_cone+def::g_input)
    , m_coneOutput(m_nh, def::g_cone+def::g_output)
{ }


OutputUavStream* Master::getReachableOutput(const def::BoardName& destination) 
{
    OutputUavStream* outputUavStream = Slave::getReachableOutput(destination);
    if( nullptr != outputUavStream ) { return outputUavStream; } 
    else if ( m_coneInput.isReachable(destination) ) { return &m_coneOutput; } //FIXME: potential error. Bad interface. Probably return weak_ptr or smth

    return nullptr;
}


bool Master::isTopicStreamed(const def::TopicName& topicName) 
{
    if( Slave::isTopicStreamed(topicName) ) { return true; }
    return m_coneInput.contains(topicName);
}