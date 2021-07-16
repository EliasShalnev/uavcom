#include "com_sim/ComSim.h"

std::string ComSim::bomber = "/bomber";
std::string ComSim::scout = "/scout";


ComSim::ComSim(const def::BoardName& boardName,
               const IOType ioType,
               ComSimObserver& comSimObserver) 
    : m_boardName(boardName)
    , m_IOType(ioType)
    , m_nh(boardName + '/' + def::g_uavNodeName)
    , m_observer(comSimObserver)
    , m_input( m_nh.advertise<uavcom::UavMessage>(getInputTopicName(ioType), 10) )
    , m_output( m_nh.subscribe<uavcom::UavMessage>(getOutputTopicName(ioType), 10,
                                                   &ComSim::ouputHandle, this) )
    , m_coordinates(m_nh, boardName+"/mavros/local_position/pose", 10)
{ }


void ComSim::publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage) 
{ 
    simulateDelay(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage) 
    {
        m_input.publish(uavMessage);
    });
}


geometry_msgs::PoseStamped::ConstPtr ComSim::getCoordinates() const 
{ 
    return m_coordinates.getMessage(); 
}


void ComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    simulateDelay(uavMessage, [this](const uavcom::UavMessage::ConstPtr& uavMessage) 
    {
        m_observer.publishToInput(m_boardName, uavMessage);
    });
}

void ComSim::simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                           const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func)
{
    ros::Timer* timer = new ros::Timer;
    
    auto callback = [uavMessage, func, timer](const ros::TimerEvent &event)
    {
        func(uavMessage);
        delete timer;
    };

    *timer = m_nh.createTimer(m_delay, callback, true);
}


def::TopicName ComSim::getOutputTopicName(const IOType ioType) 
{
    if(ioType == ComSim::IOType::Master)
    {
       return def::g_cone+def::g_output;
    }
    else if(ioType == ComSim::IOType::Slave)
    {
        return def::g_output;
    }
    else { return std::string(); }
}


def::TopicName ComSim::getInputTopicName(const IOType ioType) 
{
    if(ioType == ComSim::IOType::Master)
    {
       return def::g_cone+def::g_input;
    }
    else if(ioType == ComSim::IOType::Slave)
    {
        return def::g_input;
    }
    else { return std::string(); }   
}
