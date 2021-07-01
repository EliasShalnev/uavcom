#pragma once

#include <functional>

#include <ros/ros.h>

#include "com_sim/UavComSimStore.h"

#include "uavcom/UavMessage.h"

class UavComSim
{
public:
    UavComSim(ros::NodeHandle nodeHandle, 
              UavComSimStore& comSimStore,
              const std::string& ns);
    UavComSim(const UavComSim& origin) = delete;
    UavComSim& operator=(const UavComSim& origin) = delete;
    ~UavComSim() = default;

    void setDelay(const ros::Duration& delay) { m_delay = delay; }

    void connect() { m_connectionState = true; }

    void disconnect() { m_connectionState = false; }

    void publishToInput(const uavcom::UavMessage::ConstPtr& uavMessage);

private:
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    void simulateComChannel(const uavcom::UavMessage::ConstPtr& uavMessage,
                            const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func);

private:
    const std::string m_namespace;
    ros::NodeHandle m_nodeHandle;

    UavComSimStore& m_comSimStore;

    ros::Publisher  m_input;
    ros::Subscriber m_output;

    ros::Duration m_delay {0};
    bool m_connectionState = true;
};
