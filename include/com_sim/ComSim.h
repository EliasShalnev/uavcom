#pragma once

#include <functional>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/PoseStamped.h>
#include "uavcom/UavMessage.h"

#include "common/globals.h"

#include "com_sim/SubMonitor.h"

class ComSimObserver;

class ComSim
{
public:
    using Ptr = std::shared_ptr<ComSim>;
    using IOName = std::string;
    static std::string bomber;
    static std::string scout;
    enum IOType {Master=0, Slave};
    static std::string IOTypeToStr(const IOType& ioType);

public:
    ComSim(const def::BoardName& boardName,
           const IOType ioType,
           ComSimObserver& comSimObserver);
    ComSim(const ComSim&) = delete;
    ComSim& operator=(const ComSim&) = delete;
    virtual ~ComSim() = default;

    void publishToInput(const ComSim::Ptr from, const uavcom::UavMessage::ConstPtr& uavMessage);

    geometry_msgs::PoseStamped::ConstPtr getCoordinates() const;
    
    IOType getIOType() const { return m_ioType; }

protected:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    virtual bool check(const ComSim::Ptr from) = 0;

    void setDelay(const ros::Duration& delay) { m_delay=delay; }

    void simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                       const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func);

    double evalDistance(const ComSim* from, const ComSim* to) const;

    bool isSlaveInCone(const ComSim* master, const ComSim* slave) const;

protected:
    const IOName m_ioName;
    ros::NodeHandle m_nh;

    ros::Publisher  m_input;
    ros::Subscriber m_output;

private:
    const IOType m_ioType;

    ComSimObserver& m_observer;

    ros::Duration m_delay {0};

    SubMonitor<geometry_msgs::PoseStamped> m_coordinates;
};
