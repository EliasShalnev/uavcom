#pragma once

#include <functional>
#include <vector>

#include <ros/node_handle.h>
#include <ros/timer.h>
#include <ros/master.h>


class TopicMonitor
{
public:
    using RequestCallback = std::function<void(const XmlRpc::XmlRpcValue&)>;

public:
    TopicMonitor();
    TopicMonitor(const TopicMonitor&) = delete;
    TopicMonitor& operator=(const TopicMonitor&) = delete;
    ~TopicMonitor() = default;

    void onPublishedTopics(const RequestCallback& callback);
    void onSubscribedTopics(const RequestCallback& callback);

protected:
    virtual void systemStateRequest();

private:
    void timerCallback(const ros::TimerEvent &event);

private:
    ros::NodeHandle m_nodeHandle;
    ros::Duration m_timerFreq {3};
    ros::Timer m_requestTimer;

    std::vector<RequestCallback> m_onPublishedTopics;
    std::vector<RequestCallback> m_onSubscribedTopics;

    const std::string m_getSystemState = "getSystemState";
};

