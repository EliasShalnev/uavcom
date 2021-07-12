#pragma once

#include <ros/ros.h>

template <class MessageType>
class SubMonitor
{
public:
    SubMonitor(ros::NodeHandle nodeHandle, const std::string &topic, uint32_t queue_size)
        : m_subscriber(nodeHandle.subscribe<MessageType>
                       (topic, queue_size, &SubMonitor::callback, this) )
        , m_currentMessage(new const MessageType)
    { }
    SubMonitor(const SubMonitor &origin) = delete;
    SubMonitor& operator=(const SubMonitor &origin) = delete;
    virtual ~SubMonitor() = default;

    boost::shared_ptr<MessageType const> getMessage() const 
    { 
        return m_currentMessage; 
    }

protected:
    virtual void callback(const boost::shared_ptr<MessageType const> &message) 
    { 
        m_currentMessage = message;
    }

private:
    ros::Subscriber m_subscriber;
    boost::shared_ptr<MessageType const> m_currentMessage;
};