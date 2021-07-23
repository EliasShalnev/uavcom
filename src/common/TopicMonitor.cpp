#include "common/TopicMonitor.h"

#include <ros/this_node.h>


TopicMonitor::TopicMonitor() 
    : m_nodeHandle()
    , m_requestTimer(m_nodeHandle.createTimer(m_timerFreq,
                                              &TopicMonitor::timerCallback,
                                              this, false, false))
{ 
    systemStateRequest();
    m_requestTimer.start();
}


void TopicMonitor::systemStateRequest() 
{
    XmlRpc::XmlRpcValue params( ros::this_node::getName() ), response, payload;

    if( !ros::master::execute(m_getSystemState, params, response, payload, true) )
    {
        std::cout << "XML RPC request was failled!" << std::endl;
        return;
    }

    auto& publishedTopics = payload[0];
    for(auto callback : m_onPublishedTopics)
    {
        callback(publishedTopics);
    }

    auto& subscribedTopics = payload[1];
    for(auto callback : m_onSubscribedTopics)
    {
        callback(subscribedTopics);
    }
}


void TopicMonitor::onPublishedTopics(const RequestCallback& callback) 
{
    m_onPublishedTopics.emplace_back(callback);
}


void TopicMonitor::onSubscribedTopics(const RequestCallback& callback) 
{
    m_onSubscribedTopics.emplace_back(callback);
}


void TopicMonitor::timerCallback(const ros::TimerEvent &event) 
{
    systemStateRequest();
}
