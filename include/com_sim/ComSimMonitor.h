#pragma once

#include <ros/ros.h>

#include "com_sim/ComSimObserver.h"

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

private:
    static ComSimMonitor* m_singleton;

    ros::NodeHandle m_nodeHandle;
    ros::Timer m_requestTimer;

    ComSimObserver m_comSimObserver;

    const std::string m_getSystemState = "getSystemState";
};

