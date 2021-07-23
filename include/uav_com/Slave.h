#pragma once

#include "uavcom/StreamTopic.h"

#include "uav_com/UavCom.h"
#include "uav_com/OutputUavStream.h"
#include "uav_com/InputUavStream.h"



class Slave : public UavCom
{
public:
    Slave(const def::BoardName& boardName);
    Slave(const Slave&) = delete;
    Slave& operator=(const Slave&) = delete;
    virtual ~Slave() = default;

    void redirectToOutput(const def::TopicName& topicName) override;

    void streamTopicRequest(const def::TopicName& topicName) override;

protected:
    OutputUavStream* getReachableOutput(const def::BoardName& destination) override;

protected:
    InputUavStream  m_input;
    OutputUavStream m_output;

    const std::string STREAM_TOPIC_SRV_NAME = "stream_topic_service";

    /****Server****/
    class StreamTopicServer{
    public:
        StreamTopicServer(Slave* enclose);
        StreamTopicServer(const StreamTopicServer&) = delete;
        StreamTopicServer& operator=(const StreamTopicServer&) = delete;
        ~StreamTopicServer() = default;

    private:
        inline bool isDestExist(const def::TopicName& topicName, 
                                const def::BoardName& dest) const;
        bool onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                  uavcom::StreamTopic::Response& res);

    private:
        Slave* m_enclose;
        ros::ServiceServer m_server; //starting streaming requested topic to "output" topic
        std::unordered_multimap<def::TopicName, def::BoardName> m_destinations;
    } m_streamTopicServer;

    /****Client****/
    class StreamTopicClient{
    public:
        StreamTopicClient(Slave* enclose);
        StreamTopicClient(const StreamTopicClient&) = delete;
        StreamTopicClient& operator=(const StreamTopicClient&) = delete;
        ~StreamTopicClient() = default;

        void streamTopicRequest(const def::TopicName& topicName);

    private:
        Slave* m_enclose;
    } m_streamTopicClient;
};
