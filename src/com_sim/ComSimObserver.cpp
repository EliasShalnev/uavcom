#include "com_sim/ComSimObserver.h"

#include <memory>

#include "com_sim/ComSim.h"
#include "com_sim/SlaveChecker.h"

ComSimObserver::~ComSimObserver() 
{
    for(auto uavComSimIt : m_store) { delete uavComSimIt.second; }
}


void ComSimObserver::publishToInput(const def::BoardName& from, 
                                    const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    std::unique_ptr<ComChecker> comChecker(nullptr);
    if( from.find(ComSim::bomber) ) { comChecker.reset(new SlaveChecker); }
    else if( from.find(ComSim::scout) ) { /*checkAsScout*/ }
    else { ROS_ERROR_STREAM("Unknown uav type " << from); return; }
    
    auto fromComSim = m_store.find(from);
    
    for(auto toComSim : m_store)
    {
        if(from == toComSim.first) { continue; }

        if(comChecker == nullptr) { continue; }
        if( comChecker->check(*fromComSim->second, *toComSim.second) )
        {
            toComSim.second->publishToInput(uavMessage);
        }
    }
}


void ComSimObserver::checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics) 
{
    for(int topicIndex=0; topicIndex < publishedTopics.size(); ++topicIndex)
    {
        auto topicName = def::TopicName( publishedTopics[topicIndex][0] );
        // std::string ioPrefix;
        ComSim::IOType ioType;

        //проверка топика на /cone_output
        auto outputPos = topicName.rfind('/'+def::g_cone+def::g_output);
        if( std::string::npos != outputPos ) { ioType = ComSim::IOType::Master; }
        else 
        {
            //проверка топика на /output
            outputPos = topicName.rfind('/'+def::g_output);
            if(std::string::npos != outputPos) { ioType = ComSim::IOType::Slave; }
            else { continue; }
        }

        //проверка топика на /uav_com
        auto uav_comPos = topicName.rfind(def::g_uavNodeName, outputPos);
        if( std::string::npos == uav_comPos ) { continue; }

        //проверка на тип БпЛА. Если в префиксе нет /scout или /bomber - отбрасываем топик
        def::BoardName boardName = def::getFirstSegment(topicName);
        if(ioType == ComSim::IOType::Master)
        {
            //если топик был определен как Master, то у него должен быть префикc /scout
            if(boardName.find(ComSim::scout) == std::string::npos) { 
                ROS_ERROR_STREAM("Master should be with " << ComSim::scout << " prefix." );
                continue; 
            }
        }
        else if(ioType == ComSim::IOType::Slave)
        {
            //если топик был определен как Slave, то у него должен быть префикс /bomber
            if(boardName.find(ComSim::bomber) == std::string::npos &&
               boardName.find(ComSim::scout) == std::string::npos) { 
                ROS_ERROR_STREAM("Slave should be with " << ComSim::bomber << " prefix." );
                continue; 
            }
        }
        else { ROS_ERROR_STREAM("Unknown type " << ioType; ); }

        if( m_store.end() != m_store.find(boardName) ) { continue; }

        ROS_INFO_STREAM("find " << topicName);
        m_store.emplace( boardName, new ComSim(boardName, ioType, *this) );
    }
}
