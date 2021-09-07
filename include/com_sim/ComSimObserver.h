#pragma once

#include <unordered_map>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include "uavcom/UavMessage.h"

#include "com_sim/SubMonitor.h"
#include "com_sim/ComSim.h"


class ComSimObserver
{
public:
    ComSimObserver();
    ComSimObserver(const ComSimObserver& orig) = delete;
    ComSimObserver& operator=(const ComSimObserver& orig) = delete;
    ~ComSimObserver() = default;

    void publishToInput(const ComSim::IOName& fromIoName,
                        const uavcom::UavMessage::ConstPtr& uavMessage);

private:
     /**
     * @brief /gazebo/model_states callback
     * 
     * @param modelStates 
     */
    void checkRegisteredUavModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates);

    /**
     * @brief checks if new uav node is spawned and saves it in a bobmbers store.
     * 
     * @return std::vector<std::string> found board names
     */
    std::vector<std::string> checkRegisteredUavNodes();

private:
    std::size_t IO_TOPIC_SIZE = 3;
    std::unordered_map<ComSim::IOName, ComSim::Ptr> m_store;
    ros::NodeHandle m_nh;
    ros::Subscriber m_modelStatesSub;
};

