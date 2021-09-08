#include "com_sim/ComSim.h"

#include "com_sim/ComSimObserver.h"


std::string ComSim::IOTypeToStr(const IOType& ioType)
{
    if(ioType == ComSim::Master) { return std::string("master"); }
    else if(ioType == ComSim::Slave) { return std::string("slave"); }
}


ComSim::ComSim(Model::ModelName modelName,
               const def::BoardName& boardName,
               ComSimObserver& comSimObserver) 
    : Model(modelName)
    , m_boardName(boardName)
    , m_nh(boardName + '/' + def::g_uavNodeName)
    , m_observer(comSimObserver)
{ }


void ComSim::publishToInput(const ComSim::Ptr& from,
                            const uavcom::UavMessage::ConstPtr& uavMessage) 
{ 
    if( !check(from) ) { return; }
    
    simulateDelay(uavMessage, getDelay(from), 
                  [this](const uavcom::UavMessage::ConstPtr& uavMessage) 
    {
        m_input.publish(uavMessage);
    });
}


void ComSim::ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    m_observer.publishToInput(getIOName(), uavMessage);
}


double ComSim::getDelay(const ComSim::Ptr& from) 
{
    double delay = 0.002; //1мс - задержка по-умолчанию

    auto distance = evalDistance(from.get(), this);

    auto time = 1/std::pow(distance, 3) + delay;

    // ROS_INFO_STREAM("time " << time << " olol " << 1/std::pow(distance, 3));

    return delay;
}


void ComSim::simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                           const double& delay, 
                           const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func)
{
    ros::Timer* timer = new ros::Timer;
    
    auto callback = [uavMessage, func, timer](const ros::TimerEvent &event)
    {
        func(uavMessage);
        delete timer;
    };

    *timer = m_nh.createTimer(ros::Duration(delay), callback, true);
}


double ComSim::evalDistance2D(const ComSim* from, const ComSim* to) const 
{
    auto fromCoord = from->getCoordinates();
    auto toCoord = to->getCoordinates();     

    double fromX = fromCoord->x;
    double toX = toCoord->x;
    double deltaX = fromX-toX;

    double fromY = fromCoord->y;
    double toY = toCoord->y;
    double deltaY = fromY-toY;
        
    return std::sqrt( std::pow(deltaX, 2) + std::pow(deltaY, 2) );
}


double ComSim::evalDistance(const ComSim* from, const ComSim* to) const 
{
    auto fromCoord = from->getCoordinates();
    auto toCoord = to->getCoordinates();     

    double fromX = fromCoord->x;
    double toX = toCoord->x;
    double deltaX = fromX-toX;

    double fromY = fromCoord->y;
    double toY = toCoord->y;
    double deltaY = fromY-toY;

    double fromZ = fromCoord->z;
    double toZ = toCoord->z;
    double deltaZ = fromZ-toZ;

    return std::sqrt( std::pow(deltaX, 2) + std::pow(deltaY, 2)  + std::pow(deltaZ, 2) );
}


bool ComSim::isSlaveInCone(const ComSim* master, const ComSim* slave) const
{
    auto masterCoord = master->getCoordinates();
    if(masterCoord == nullptr) { return false; }
    auto slaveCoord = slave->getCoordinates();
    if(slaveCoord == nullptr) { return false; }

    double masterHeight = masterCoord->z;
    double slaveHeight = slaveCoord->z;

    //slave height should be lesser then master's height with half meter reserve
    if(slaveHeight+0.5 >= masterHeight) { return false; }

    double deltaHeight = masterHeight - slaveHeight;

    //TODO - hardcode! Angle should be dynamicly set
    constexpr double pi = 3.14159265;
    double radius = tan(45*pi/180 )*masterHeight;
    double subRadius = (radius*deltaHeight)/masterHeight;
    double distance = evalDistance2D(master, slave);

    if(subRadius < distance)
    {
        ROS_ERROR_STREAM( "Slave \"" << slave->getIOName() << 
                          "\" is not in master's cone \"" << master->getIOName() << "\"." );
        ROS_ERROR_STREAM( "Distance \"" << distance << "\" should be lesser than \"" 
                                        << subRadius << "\". radius: " << radius );
    }

    return subRadius >= distance;
}