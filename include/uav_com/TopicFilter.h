#pragma once

#include <memory>

#include "uav_com/UavCom.h"


class TopicFilter
{
public:
    TopicFilter(const def::BoardName& boardName);
    ~TopicFilter() = default;

    void checkPublishedTopics(const XmlRpc::XmlRpcValue& pubTopics);

    void checkSubscribedTopics(const XmlRpc::XmlRpcValue& subTopics);

private:
    void forEachUavComTopic(const XmlRpc::XmlRpcValue &topics,
                            const std::function<void(const def::TopicName&)>& UnaryFunc);

    inline bool isUserNodeExists(const XmlRpc::XmlRpcValue &nodeNames) const;

private:
    const std::size_t MIN_TOPIC_SIZE = 4;
    std::unique_ptr<UavCom> m_uavCom = nullptr;

};
