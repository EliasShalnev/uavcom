#include "uav_com/OutputUavStream.h"

#include "uavcom/Heartbeat.h"

#include "uav_com/UavComMonitor.h"


namespace def {

ros::Publisher OutputUavStream::m_heartbeatPub = def::UavComMonitor::getInstance()->getNodeHandle()
                                                 .advertise<uavcom::Heartbeat>(ros::this_node::getName() + BROADCAST + 
                                                                               ros::this_node::getNamespace() + HEARTBEAT, 10);

ros::Timer OutputUavStream::m_heartbeatTimer = def::UavComMonitor::getInstance()->getNodeHandle()
                                               .createTimer(ros::Duration(0.2),
                                                            &OutputUavStream::publishHeartBeat);


OutputUavStream::OutputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName)
    : m_nodeHandle(nodeHandle)
    , m_output( nodeHandle.advertise<uavcom::UavMessage>(ros::this_node::getName() + '/' + streamName, 10) )
    , m_subCheckTimer( nodeHandle.createTimer(ros::Duration(5), 
                                              &OutputUavStream::checkSubscribers,
                                              this) )
    // , m_heartbeatPub( nodeHandle.advertise<uavcom::Heartbeat>(ros::this_node::getName() + BROADCAST + 
    //                                                           ros::this_node::getNamespace() + HEARTBEAT, 10) )
    // , m_heartbeatTimer( nodeHandle.createTimer(ros::Duration(0.2),
                                            //    &OutputUavStream::publishHeartBeat,
                                            //    this) )
{ 
    const std::string heartBeatTopicName = ros::this_node::getName() + BROADCAST + 
                                           ros::this_node::getNamespace() + HEARTBEAT;
    redirectToOutput(heartBeatTopicName);
}

void OutputUavStream::redirectToOutput(const std::string& topicName) 
{
    //check if published topic have already subscribed by this node
    if( m_toOutputTopics.end() != m_toOutputTopics.find(topicName) ) { return; }

    ROS_INFO_STREAM("Subscribing to topic: " << topicName);

    boost::function<void(const RosMsgParser::ShapeShifter::ConstPtr&)> callback;
    callback = [this, topicName](const RosMsgParser::ShapeShifter::ConstPtr &msg)
    {
        //preparing output topic msg
        uavcom::UavMessage::Ptr uavMessage(new uavcom::UavMessage);
        uavMessage->topicName = getRemoteTopicName(topicName);
        uavMessage->from = ros::this_node::getNamespace();
        uavMessage->MD5Sum = msg->getMD5Sum();
        uavMessage->dataType = msg->getDataType();
        getByteArray( *msg, uavMessage->byteArray );

        //TODO: check if it reachable
        m_output.publish(uavMessage);
    };

    ros::Subscriber subscriber = m_nodeHandle.subscribe(topicName, 10, callback);
    m_toOutputTopics.emplace(topicName, subscriber);
}


bool OutputUavStream::contains(const TopicName& topicName) const
{
    return m_toOutputTopics.end() !=  m_toOutputTopics.find(topicName);
}


void OutputUavStream::publish(const uavcom::UavMessage::Ptr& msg) 
{
    m_output.publish(msg);
}


void OutputUavStream::checkSubscribers(const ros::TimerEvent& event)
{
    for( auto it = m_toOutputTopics.begin(); it != m_toOutputTopics.end(); )
    {
        if(it->second.getNumPublishers() == 0)
        {
            auto tpName = it->first;

            it = m_toOutputTopics.erase(it);
            ROS_INFO_STREAM("Unsubscribing from: " << tpName);
        }
        else { ++it; }
    }
}


void OutputUavStream::publishHeartBeat(const ros::TimerEvent& event) 
{
    uavcom::Heartbeat heartbeat;
    heartbeat.from = ros::this_node::getNamespace();

    m_heartbeatPub.publish(heartbeat);
}


} //namespace def