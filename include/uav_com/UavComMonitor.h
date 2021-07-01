#pragma once

#include <ros/ros.h>

#include "uav_com/OutputMonitor.h"
#include "uav_com/InputMonitor.h"


class UavComMonitor final
{
public:
    static UavComMonitor* getInstance();
    UavComMonitor(const UavComMonitor& origin) = delete;
    UavComMonitor& operator=(const UavComMonitor& origin) = delete;
    ~UavComMonitor() = default;

    void start(const ros::Duration &duration);

    void stop();

private:
    UavComMonitor();

    void requestTimerCallback(const ros::TimerEvent &event);

    void systemStateRequest();

    void forEachUavComTopic(const XmlRpc::XmlRpcValue &topics,
                            const std::function<void(const std::string&)>& UnaryFunc);

private:
    inline bool isUserNodeExists(const XmlRpc::XmlRpcValue &nodeNames) const;

private:
    static UavComMonitor* m_singleton;
    
    ros::NodeHandle m_nodeHandle;

    ros::Timer m_stateRequestTimer;

    const std::string m_getSystemState = "getSystemState";

    OutputMonitor m_outputMonitor;
    InputMonitor  m_inputMonitor;
};

