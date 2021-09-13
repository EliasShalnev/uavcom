#pragma once

#include <functional>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "uavcom/UavMessage.h"

#include "common/globals.h"

#include "com_sim/Model.h"
#include "com_sim/SubMonitor.h"

class ComSimObserver;

class ComSim : public Model
{
public:
    using Ptr = std::shared_ptr<ComSim>;
    using IOName = std::string;
    static constexpr char bomber[] = "/bomber";
    static constexpr char scout[] = "/scout";
    static constexpr char ral_x6[] = "ral_x6";
    static constexpr char orlanCam[] = "orlan_cam";
    static constexpr char orlan[] = "orlan";
    enum IOType {Master=0, Slave};
    static std::string IOTypeToStr(const IOType& ioType);

public:
    ComSim(Model::ModelName modelName,
           const def::BoardName& boardName,
           ComSimObserver& comSimObserver);
    ComSim(const ComSim&) = delete;
    ComSim& operator=(const ComSim&) = delete;
    virtual ~ComSim() = default;

    void publishToInput(const ComSim::Ptr& from, const uavcom::UavMessage::ConstPtr& uavMessage);

    virtual IOType getIOType() const = 0; 

    virtual IOName getIOName() const = 0;

protected:
    virtual bool check(const ComSim::Ptr from) const = 0;
    
    void ouputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    double getDelay(const ComSim::Ptr& from);

    void simulateDelay(const uavcom::UavMessage::ConstPtr& uavMessage,
                       const double& delay, 
                       const std::function<void(const uavcom::UavMessage::ConstPtr&)>& func);

    double evalDistance2D(const ComSim* from, const ComSim* to) const;

    double evalDistance(const ComSim* from, const ComSim* to) const ;

    bool isSlaveInCone(const ComSim* master, const ComSim* slave) const;

protected:
    const def::BoardName m_boardName;
    ros::NodeHandle m_nh;

    ros::Publisher  m_input;
    ros::Subscriber m_output;

private:
    ComSimObserver& m_observer;
};
