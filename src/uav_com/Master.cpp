#include "uav_com/Master.h"


Master::Master(const def::BoardName& boardName) 
    : Slave(boardName)
    , m_masterInput( new InputUavStream(m_nh, def::g_masterIOPrefix+def::g_input) )
    , m_masterOutput( new OutputUavStream(m_nh, def::g_masterIOPrefix+def::g_output) )
{ }


OutputUavStream::Ptr Master::getReachableOutput(const def::BoardName& destination) 
{
    auto outputUavStream = Slave::getReachableOutput(destination);
    if( nullptr != outputUavStream ) { return outputUavStream; } 
    else if ( m_masterInput->isReachable(destination) ) { return m_masterOutput; }

    return nullptr;
}


bool Master::isTopicStreamed(const def::TopicName& topicName) 
{
    if( Slave::isTopicStreamed(topicName) ) { return true; }
    return m_masterInput->contains(topicName);
}