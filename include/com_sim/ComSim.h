#pragma once

#include <functional>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "common/common.h"

#include <geometry_msgs/PoseStamped.h>
#include "uavcom/UavMessage.h"

#include "com_sim/ComSimObserver.h"
#include "com_sim/SubMonitor.h"

class ComSim
{
public:
    static std::string bomber;
    static std::string scout;
    enum IOType {Master=0, Slave};

public:
    ComSim(const def::TopicName& topicName,
           const IOType ioType,
           ComSimObserver& comSimObserver);
    ComSim(const ComSim&) = delete;
    ComSim& operator=(const ComSim&) = delete;
    ~ComSim() = default;

    void publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage);

    void setDelay(const ros::Duration& delay) { m_delay=delay; }

    IOType getIOType() const { return m_IOType; }

    geometry_msgs::PoseStamped::ConstPtr getCoordinates() const;

private:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    void simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                       const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func);

    static def::TopicName getOutputTopicName(const IOType ioType);
    static def::TopicName getInputTopicName(const IOType ioType);

private:
    const def::BoardName m_boardName;
    const IOType m_IOType;
    ros::NodeHandle m_nh;

    ComSimObserver& m_observer;

    ros::Publisher  m_input;
    ros::Subscriber m_output;

    ros::Duration m_delay {0};

    SubMonitor<geometry_msgs::PoseStamped> m_coordinates;
};
