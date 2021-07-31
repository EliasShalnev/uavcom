#include "com_sim/ComSim.h"

#include "com_sim/ComSimObserver.h"

std::string ComSim::bomber = "/bomber";
std::string ComSim::scout = "/scout";



std::string ComSim::IOTypeToStr(const IOType& ioType)
{
    if(ioType == ComSim::Master) { return std::string("master"); }
    else if(ioType == ComSim::Slave) { return std::string("slave"); }
}


ComSim::ComSim(const def::BoardName& boardName,
               const IOType ioType,
               ComSimObserver& comSimObserver) 
    : m_ioName( boardName +'/' + ComSim::IOTypeToStr(ioType) )
    , m_ioType(ioType)
    , m_nh(boardName + '/' + def::g_uavNodeName)
    , m_observer(comSimObserver)
    , m_coordinates(m_nh, boardName+"/mavros/local_position/pose", 10)
{ }


void ComSim::publishToInput(const ComSim::Ptr from,
                            const uavcom::UavMessage::ConstPtr& uavMessage) 
{ 
    if( !check(from) ) { return; }
    
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
        m_observer.publishToInput(m_ioName, uavMessage);
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


double ComSim::evalDistance(const ComSim* from, const ComSim* to) const 
{
    auto fromCoord = from->getCoordinates();
    auto toCoord = to->getCoordinates();     

    double fromX = fromCoord->pose.position.x;
    double toX = toCoord->pose.position.x;
    double deltaX = fromX-toX;

    double fromY = fromCoord->pose.position.y;
    double toY = toCoord->pose.position.y;
    double deltaY = fromY-toY;
        
    return std::sqrt( ( std::pow(deltaX, 2) + std::pow(deltaY, 2) ) );
}


bool ComSim::isSlaveInCone(const ComSim* master, const ComSim* slave) const
{
    auto masterCoord = master->getCoordinates();
    if(masterCoord == nullptr) { return false; }
    auto slaveCoord = slave->getCoordinates();
    if(slaveCoord == nullptr) { return false; }

    double masterHeight = masterCoord->pose.position.z;
    double slaveHeight = slaveCoord->pose.position.z;

    //slave height should be lesser then master height 
    if(slaveHeight >= masterHeight) { return false; }    

    double deltaHeight = masterHeight - slaveHeight;

    //TODO - hardcode! Angle should be dynamicly set
    double radius = tan(45)*masterHeight;
    double subRadius = (radius*deltaHeight)/masterHeight;
    double distance = evalDistance(master, slave); 


    return subRadius >= distance;
}
