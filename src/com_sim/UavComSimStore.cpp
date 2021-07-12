#include "com_sim/UavComSimStore.h"

#include "com_sim/UavComSim.h"

UavComSimStore::~UavComSimStore() 
{
    for(auto uavComSimIt : m_store) { delete uavComSimIt.second; }
}


void UavComSimStore::publishToInput(const std::string& ns, 
                                    const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    auto uavComSimIt = m_store.find(ns);
    if( m_store.end() != uavComSimIt )
    {
        uavComSimIt->second->publishToInput(uavMessage);
    }
}


void UavComSimStore::publishToAll(const std::string& nameSpace,
                                  const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    for(auto [ns, uavComSim] : m_store)
    {   
        if(nameSpace == ns) { continue; }
        uavComSim->publishToInput(uavMessage);
    }
}


geometry_msgs::PoseStamped::ConstPtr UavComSimStore::getCoordinates(const std::string& ns) 
{
    geometry_msgs::PoseStamped::ConstPtr result = nullptr;
    auto uavComSimIt = m_store.find(ns);
    if( m_store.end() != uavComSimIt )
    {
        result = uavComSimIt->second->getCoordinates();
    }
    return result;
}

