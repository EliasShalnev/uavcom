#include "com_sim/ComSimObserver.h"

#include <ros/master.h>

#include "common/NSParser.h"

#include "com_sim/ComSim.h"
#include "com_sim/SlaveComSim.h"
#include "com_sim/MasterComSim.h"


ComSimObserver::ComSimObserver()
    : m_modelStatesSub( m_nh.subscribe<gazebo_msgs::ModelStates>
                        ("/gazebo/model_states", 10, &ComSimObserver::checkRegisteredUavModels, this) )
{ }


void ComSimObserver::publishToInput(const ComSim::IOName& fromIoName, 
                                    const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    auto fromComSim = m_store.find(fromIoName);

    for(auto toComSim : m_store)
    {
        if(fromIoName == toComSim.first) { continue; }

        toComSim.second->publishToInput(fromComSim->second, uavMessage);
    }
}


void ComSimObserver::checkRegisteredUavModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    auto boardNames = checkRegisteredUavNodes();

    for(auto boardName : boardNames)
    {
        ComSim::IOType ioType;
        std::string id;
        //Извлекаем id и тип БпЛА
        if(boardName.find(ComSim::bomber) != std::string::npos) 
        {
            id = boardName.substr( strlen(ComSim::bomber) );
            ioType = ComSim::IOType::Slave; 
        }
        else if(boardName.find(ComSim::scout) != std::string::npos)
        {
            id = boardName.substr( strlen(ComSim::scout) );
            ioType = ComSim::IOType::Master; 
        }
        
        for(auto model : modelStates->name)
        {
            std::string modelId;
            //извлекаем id из модели ral_x6 или orlan
            if(model.find(ComSim::ral_x6) != std::string::npos) 
            {
                modelId = model.substr( strlen(ComSim::ral_x6) );
            }
            else if(model.find(ComSim::orlanCam) != std::string::npos)
            {
                modelId = model.substr( strlen(ComSim::orlanCam) );
            }
            else if(model.find(ComSim::orlan) != std::string::npos)
            {
                modelId = model.substr( strlen(ComSim::orlan) );
            }
            else { continue; }
            
            //если id совпадают, то создаем наследника ComSim исходя из найденого типа БпЛА
            if(id != modelId) { continue; }
            
            if(ioType == ComSim::IOType::Master)
            {
                /* slave interface creation */
                const std::string slaveInterface = boardName+'/'+ComSim::IOTypeToStr(ComSim::IOType::Slave);
                if( m_store.end() == m_store.find(slaveInterface) ) 
                { 
                    ROS_INFO_STREAM("Found " << slaveInterface << " with model \"" << model << "\".");
                    ComSim* slaveComSim = new SlaveComSim(model, boardName, *this);
                    m_store.emplace(slaveInterface, slaveComSim);
                }
                /* master interface creation */
                const std::string masterInterface = boardName+'/'+ComSim::IOTypeToStr(ComSim::IOType::Master);
                if(m_store.end() == m_store.find(masterInterface))
                {
                    ROS_INFO_STREAM("Found " << masterInterface << " with model \"" << model << "\".");
                    ComSim* masterComSim = new MasterComSim(model, boardName, *this);
                    m_store.emplace(masterInterface, masterComSim);
                }
            }
            else if(ioType == ComSim::IOType::Slave)
            {
                const std::string interface = boardName+'/'+ComSim::IOTypeToStr(ComSim::IOType::Slave);
                if( m_store.end() != m_store.find(interface) ) { continue; }

                ROS_INFO_STREAM("Found " << interface << " with model \"" << model << "\".");
                ComSim* slaveComSim = new SlaveComSim(model, boardName, *this);
                m_store.emplace(interface, slaveComSim);  
            }
        }
    }
}


std::vector<std::string> ComSimObserver::checkRegisteredUavNodes() 
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    std::vector<std::string> uavs;

    for(auto node : nodes)
    {
        NSParser nsParser(node);
        if(nsParser.size() != 2) { continue; }

        if( *(nsParser.end()-1) != std::string("/mavros") ) { continue; }
        
        const std::string boardName = *nsParser.begin();

        if( boardName.find(ComSim::bomber) != std::string::npos &&
            boardName.find(ComSim::scout) != std::string::npos ) 
        {
            continue;
        }
        else { uavs.emplace_back(boardName); }
    }
    return uavs;
}
