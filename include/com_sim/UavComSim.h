#pragma once

#include <functional>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "uavcom/UavMessage.h"

#include "com_sim/UavComSimStore.h"
#include "com_sim/SubMonitor.h"

class UavComSim
{
public:
    UavComSim(ros::NodeHandle nodeHandle, 
              UavComSimStore& comSimStore,
              const std::string& ns);
    UavComSim(const UavComSim& origin) = delete;
    UavComSim& operator=(const UavComSim& origin) = delete;
    ~UavComSim() = default;

    geometry_msgs::PoseStamped::ConstPtr getCoordinates() const { return m_coordinates.getMessage(); }

    void publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage);

private:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    bool isUavAvailable(const std::string& ns) const;

    void simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                       const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func);

private:
    const std::string m_namespace;
    ros::NodeHandle m_nodeHandle;

    UavComSimStore& m_comSimStore;

    ros::Publisher  m_input;
    ros::Subscriber m_output;

    SubMonitor<geometry_msgs::PoseStamped> m_coordinates;

    ros::Duration m_delay {0};
};
