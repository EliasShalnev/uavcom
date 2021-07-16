#include "com_sim/ComSimMonitor.h"


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
    : m_nodeHandle()
    , m_requestTimer( m_nodeHandle.createTimer(ros::Duration(), 
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
    XmlRpc::XmlRpcValue params( ros::this_node::getName() ), response, payload;

    if( !ros::master::execute(m_getSystemState, params, response, payload, true) )
    {
        std::cout << "XML RPC request was failled!" << std::endl;
        return;
    }
    auto& publishedTopics = payload[0];

    m_uavComSimStore.checkPublishedTopics(publishedTopics);
}