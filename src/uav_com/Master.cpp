#include "uav_com/Master.h"



namespace def {


Master::Master(ros::NodeHandle& nodeHandle) 
    : Slave(nodeHandle)
    , m_coneInput(nodeHandle, g_cone+g_input)
    , m_coneOutput(nodeHandle, g_cone+g_output)
{ }


void Master::redirectToOutput(const TopicName& topicName) 
{
    Slave::redirectToOutput(topicName);
}


void Master::streamTopicRequest(const TopicName& topicName) 
{
    Slave::streamTopicRequest(topicName);
}


OutputUavStream* Master::getReachableOutput(const BoardName& destination) 
{
    OutputUavStream* outputUavStream = Slave::getReachableOutput(destination);
    if( nullptr != outputUavStream ) { return outputUavStream; } 
    else if ( m_coneInput.isReachable(destination) ) { return &m_coneOutput; } //FIXME: potential error. Bad interface. Probably return weak_ptr or smth

    return nullptr;
}


} //namespace def
