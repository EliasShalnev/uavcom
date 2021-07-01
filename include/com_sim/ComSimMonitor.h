#pragma once

#include <ros/ros.h>

#include "com_sim/UavComSimStore.h"

class ComSimMonitor final
{
public:
    static ComSimMonitor* getInstance();
    ComSimMonitor(const ComSimMonitor& origin) = delete;
    ComSimMonitor& operator=(const ComSimMonitor& origin) = delete;
    ~ComSimMonitor() = default;

    void start(const ros::Duration &duration);

    void stop();

private:
    ComSimMonitor();

    void requestTimerCallback(const ros::TimerEvent &event);

    void systemStateRequest();

    void checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics);

private:
    static ComSimMonitor* m_singleton;

    ros::NodeHandle m_nodeHandle;
    ros::Timer m_requestTimer;

    UavComSimStore m_uavComSimStore;

    const std::string m_getSystemState = "getSystemState";
};

