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
    , m_coordinates(nodeHandle, ns+"/mavros/local_position/pose", 10)
{ }


void UavComSim::publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    simulateDelay(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage)
    {
        m_input.publish(uavMessage);
    });
}


void UavComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    const std::string ns = def::getFirstSegment(uavMessage->topicName);
    // if( isUavAvailable(ns) )
    // {
        simulateDelay(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage)
        {
            m_comSimStore.publishToAll(uavMessage->from, uavMessage);
        });  
    // }
}


bool UavComSim::isUavAvailable(const std::string& ns) const
{
    auto destCoordnates = m_comSimStore.getCoordinates(ns);
    auto destHeight = destCoordnates->pose.position.z;

    auto thisCoordinates = getCoordinates();
    auto thisHeight = thisCoordinates->pose.position.z;

    if( thisHeight > destHeight )
    {
        
    }
}


void UavComSim::simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                              const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func) 
{
    ros::Timer* timer = new ros::Timer;
    
    auto callback = [uavMessage, func, timer](const ros::TimerEvent &event){
        timer->stop();
        func(uavMessage);
        delete timer;
    };

    *timer = m_nodeHandle.createTimer(m_delay, callback, true);
}

