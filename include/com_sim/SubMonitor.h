#pragma once

#include <ros/node_handle.h>
#include <ros/subscriber.h>


template <class MessageType>
class SubMonitor
{
public:
    SubMonitor(ros::NodeHandle nodeHandle, const std::string &topic, uint32_t queue_size)
        : m_subscriber(nodeHandle.subscribe<MessageType>
                       (topic, queue_size, &SubMonitor::callback, this) )
        , m_currentMessage(nullptr)
    { }
    SubMonitor(const SubMonitor &origin) = delete;
    SubMonitor& operator=(const SubMonitor &origin) = delete;
    virtual ~SubMonitor() = default;

    /**
     * @brief Get last message that have published in subscribed topic
     * 
     * @return Returns last message that have published.
     *         Returns nullptr if there are no publishers to subscribed topic.
     */
    boost::shared_ptr<MessageType const> getMessage() const 
    {
        if(m_subscriber.getNumPublishers() == 0) 
        { 
            ROS_WARN_STREAM("There is no publishers to " << m_subscriber.getTopic() << " topic.");
            return nullptr; 
        }
        if(m_currentMessage == nullptr) { ROS_WARN_STREAM("There wasn't any message in " << m_subscriber.getTopic() << " topic."); }
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