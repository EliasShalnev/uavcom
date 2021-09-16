#pragma once

#include <ros/node_handle.h>

#include "uavcom/StreamTopic.h"

#include "common/globals.h"

#include "uav_com/OutputUavStream.h"
#include "uav_com/InputUavStream.h"


class UavCom
{
public:
    const static std::string MASTER;
    const static std::string SLAVE;

public:
    UavCom(const def::BoardName& boardName);
    UavCom(const UavCom&) = delete;
    UavCom& operator=(const UavCom&) = delete;
    virtual ~UavCom() = default;

    void redirectToOutput(const def::TopicName& topicName);

    virtual void streamTopicRequest(const def::TopicName& topicName);

protected:
    virtual OutputUavStream::Ptr getReachableOutput(const def::BoardName& destination) = 0;
    
    virtual bool isTopicStreamed(const def::TopicName& topicName) = 0;

protected:
    const def::BoardName m_boardName; 
    ros::NodeHandle m_nh;

    const std::string STREAM_TOPIC_SRV_NAME = "stream_topic_service";

    /****Server****/
    class StreamTopicServer{
    public:
        StreamTopicServer(UavCom* enclose);
        StreamTopicServer(const StreamTopicServer&) = delete;
        StreamTopicServer& operator=(const StreamTopicServer&) = delete;
        ~StreamTopicServer() = default;

    private:
        inline bool isDestExist(const def::TopicName& topicName, 
                                const def::BoardName& dest) const;
        bool onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                  uavcom::StreamTopic::Response& res);

    private:
        UavCom* m_enclose;
        ros::ServiceServer m_server; //starting streaming requested topic to "output" topic
        std::unordered_multimap<def::TopicName, def::BoardName> m_destinations;
    } m_streamTopicServer;

    /****Client****/
    class StreamTopicClient{
    public:
        StreamTopicClient(UavCom* enclose);
        StreamTopicClient(const StreamTopicClient&) = delete;
        StreamTopicClient& operator=(const StreamTopicClient&) = delete;
        ~StreamTopicClient() = default;

        void streamTopicRequest(const def::TopicName& topicName);

    private:
        UavCom* m_enclose;
    } m_streamTopicClient;
};

