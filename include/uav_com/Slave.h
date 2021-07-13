#pragma once

#include "common/common.h"

#include "uavcom/StreamTopic.h"

#include "uav_com/UavCom.h"
#include "uav_com/OutputUavStream.h"
#include "uav_com/InputUavStream.h"


namespace def {

class Slave : public UavCom
{
public:
    Slave(ros::NodeHandle nodeHandle);
    Slave(const Slave& origin) = delete;
    Slave& operator=(const Slave& origin) = delete;
    virtual ~Slave() = default;

    void redirectToOutput(const TopicName& topicName) override;

    void streamTopicRequest(const TopicName& topicName) override;

protected:
    // bool containsInOutput(const TopicName& topicName) override; 
    OutputUavStream* getReachableOutput(const Destination& destination) override;

protected:
    ros::NodeHandle m_nodeHandle;
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
        inline bool isDestExist(const std::string& topicName, 
                                const std::string& dest) const;
        bool onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                                  uavcom::StreamTopic::Response& res);

    private:
        Slave* m_enclose;
        ros::ServiceServer m_server; //starting streaming requested topic to "output" topic
        std::unordered_multimap<TopicName, Destination> m_destinations;
    } m_streamTopicServer;

    /****Client****/
    class StreamTopicClient{
    public:
        StreamTopicClient(Slave* enclose);
        StreamTopicClient(const StreamTopicClient&) = delete;
        StreamTopicClient& operator=(const StreamTopicClient&) = delete;
        ~StreamTopicClient() = default;

        void streamTopicRequest(const std::string& topicName);

    private:
        Slave* m_enclose;
    } m_streamTopicClient;
};

} //namespace def