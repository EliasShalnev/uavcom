#include "com_sim/UavComSim.h"

#include "common/common.h"


UavComSim::UavComSim(ros::NodeHandle nodeHandle,  
                     UavComSimStore& comSimStore,
                     const std::string& ns) 
    : m_namespace(ns)
    , m_nodeHandle(nodeHandle)
    , m_comSimStore(comSimStore)
    , m_input( nodeHandle.advertise<uavcom::UavMessage>(ns+"/uav_com/input", 10) )
    , m_output( nodeHandle.subscribe<uavcom::UavMessage>(ns+"/uav_com/output", 10, 
                                                         &UavComSim::ouputHandle, this) )

{ }


void UavComSim::publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    simulateComChannel(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage)
    {
        m_input.publish(uavMessage);
    });
}


void UavComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    simulateComChannel(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage)
    {
        std::string ns = getFirstSerment(uavMessage->topicName);
        m_comSimStore.publishToInput(ns, uavMessage);
    });
}


void UavComSim::simulateComChannel(const uavcom::UavMessage::ConstPtr& uavMessage,
                                   const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func) 
{
    if( !m_connectionState ) { return; } //отброс сообщения в случае дисконекта

    ros::Timer* timer = new ros::Timer;
    
    auto callback = [uavMessage, func, timer](const ros::TimerEvent &event){
        timer->stop();
        func(uavMessage);
        delete timer;
    };

    *timer = m_nodeHandle.createTimer(m_delay, callback, true);
}

