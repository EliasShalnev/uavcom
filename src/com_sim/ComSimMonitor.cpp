#include "com_sim/ComSimMonitor.h"

#include "com_sim/UavComSim.h"


ComSimMonitor* ComSimMonitor::m_singleton = nullptr;


ComSimMonitor* ComSimMonitor::getInstance() 
{
    if(nullptr == m_singleton) { m_singleton = new ComSimMonitor(); }

    return m_singleton;
}


void ComSimMonitor::start(const ros::Duration &duration) 
{
    m_requestTimer.setPeriod(duration);
    m_requestTimer.start();
}


void ComSimMonitor::stop() { m_requestTimer.stop(); }


ComSimMonitor::ComSimMonitor()
    : m_requestTimer( m_nodeHandle.createTimer(ros::Duration(), 
                                              &ComSimMonitor::requestTimerCallback,
                                              this, false, false) )
{
    systemStateRequest();
    start( ros::Duration(5) );
}


void ComSimMonitor::requestTimerCallback(const ros::TimerEvent &event) 
{
    systemStateRequest();
}


void ComSimMonitor::systemStateRequest() 
{
    XmlRpc::XmlRpcValue params(ros::this_node::getName()), response, payload;

    if( !ros::master::execute(m_getSystemState, params, response, payload, true) )
    {
        std::cout << "XML RPC request was failled!" << std::endl;
        return;
    }
    auto& publishedTopics = payload[0];

    checkPublishedTopics(publishedTopics);
}


void ComSimMonitor::checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics) 
{
    for(int topicIndex=0; topicIndex < publishedTopics.size(); ++topicIndex)
    {
        auto topicName = std::string( publishedTopics[topicIndex][0] );

        auto outputPos = topicName.rfind("/output");
        if( std::string::npos == outputPos ) { continue; }

        auto uav_comPos = topicName.rfind("/uav_com", outputPos);
        if( std::string::npos == uav_comPos ) { continue; }

        std::string ns = topicName.substr( 0, uav_comPos );
        if( m_uavComSimStore.find(ns) ) { continue; }

        std::cout << "find " << ns << std::endl;
        m_uavComSimStore.emplace( ns, new UavComSim(m_nodeHandle, m_uavComSimStore, ns) );
    }
}

