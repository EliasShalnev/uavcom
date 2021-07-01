#include "uav_com/UavComMonitor.h"

#include "ros_msg_parser/ros_parser.hpp"

#include "common/TopicNameHelper.h"


UavComMonitor* UavComMonitor::m_singleton = nullptr;


UavComMonitor* UavComMonitor::getInstance() 
{
    if(nullptr == m_singleton) { m_singleton = new UavComMonitor(); }

    return m_singleton;
}


void UavComMonitor::start(const ros::Duration &duration) 
{
    m_stateRequestTimer.setPeriod(duration);
    m_stateRequestTimer.start();
}


void UavComMonitor::stop() { m_stateRequestTimer.stop(); }


UavComMonitor::UavComMonitor()
    : m_stateRequestTimer( m_nodeHandle.createTimer(ros::Duration(), 
                                                    &UavComMonitor::requestTimerCallback,
                                                    this, false, false) )
    , m_outputMonitor(m_nodeHandle)
    , m_inputMonitor(m_nodeHandle)
{
    systemStateRequest();
    start( ros::Duration(2) ); //timer tick equals 2 sec by default
}


void UavComMonitor::requestTimerCallback(const ros::TimerEvent &event) 
{
    systemStateRequest();
}


void UavComMonitor::systemStateRequest() 
{
    XmlRpc::XmlRpcValue params(ros::this_node::getName()), response, payload;

    if( !ros::master::execute(m_getSystemState, params, response, payload, true) )
    {
        ROS_WARN_STREAM("XML RPC request was failled!");
        return;
    }
    auto& publishedTopics = payload[0];
    auto& subscribedTopics = payload[1];

    forEachUavComTopic(publishedTopics, [this](const std::string& topicName)
    {
        m_outputMonitor.redirectToOutput(topicName);
    });

    forEachUavComTopic(subscribedTopics, [this](const std::string& topicName)
    {
        m_inputMonitor.streamTopicRequest(topicName);
    });
}


void UavComMonitor::forEachUavComTopic(const XmlRpc::XmlRpcValue &topics,
                                       const std::function<void(const std::string&)>& UnaryFunc) 
{
    for(int topicIndex=0; topicIndex < topics.size(); ++topicIndex)
    {
        const std::string topicName = std::string( topics[topicIndex][0] );

        //check if node name prefix is presented in the name of the topic 
        if( !isUavcomTopic(topicName) ) { continue; }

        const std::string remoteTopic = getRemoteTopicName(topicName);
        if( remoteTopic == std::string("/input") || 
            remoteTopic == std::string("/output") ) { continue; }

        //check if user is published 
        if( !isUserNodeExists( topics[topicIndex][1] ) ) 
        { 
            continue; 
        }

        UnaryFunc(topicName);
    }
}


inline bool UavComMonitor::isUserNodeExists(const XmlRpc::XmlRpcValue &nodeNames) const
{
    for(int i=0; i < nodeNames.size(); ++i)
    {
        if( std::string(nodeNames[i]) != ros::this_node::getName() ) { return true; }  
    }
    return false;
}

