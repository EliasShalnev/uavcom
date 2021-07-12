#pragma once

#include <unordered_map>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "uavcom/UavMessage.h"


class UavComSim;

class UavComSimStore
{
public:
    UavComSimStore() = default;
    UavComSimStore(const UavComSimStore& orig) = delete;
    UavComSimStore& operator=(const UavComSimStore& orig) = delete;
    ~UavComSimStore();

    void publishToInput(const std::string& ns, 
                        const uavcom::UavMessage::ConstPtr& uavMessage);

    //TODO - удалить
    void publishToAll(const std::string& nameSpace,
                      const uavcom::UavMessage::ConstPtr& uavMessage);

    inline bool find(const std::string &ns)
    {
        return (m_store.end() != m_store.find(ns));
    }

    template <typename... Args>
    bool emplace(Args&&... args)
    {
       auto [it, isEmplaced] = m_store.emplace(std::forward<Args>(args)...);
       return isEmplaced;
    }

    geometry_msgs::PoseStamped::ConstPtr getCoordinates(const std::string& ns);


private:
    std::unordered_map<std::string, UavComSim*> m_store;
};

