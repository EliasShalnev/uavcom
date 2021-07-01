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

