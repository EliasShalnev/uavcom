#include "com_sim/ComSimObserver.h"

#include "com_sim/ComSim.h"
#include "com_sim/SlaveComSim.h"
#include "com_sim/MasterComSim.h"

ComSimObserver::~ComSimObserver() 
{
    for(auto uavComSimIt : m_store) { delete uavComSimIt.second; }
}


void ComSimObserver::publishToInput(const ComSim::IOName& fromIoName, 
                                    const uavcom::UavMessage::ConstPtr& uavMessage) 
{
    auto fromComSim = m_store.find(fromIoName);
    
    for(auto toComSim : m_store)
    {
        if(fromIoName == toComSim.first) { continue; }

        toComSim.second->publishToInput( fromComSim->second, uavMessage);
    }
}


void ComSimObserver::checkPublishedTopics(const XmlRpc::XmlRpcValue &publishedTopics) 
{
    for(int topicIndex=0; topicIndex < publishedTopics.size(); ++topicIndex)
    {
        auto topicName = def::TopicName( publishedTopics[topicIndex][0] );
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

        std::string interface = boardName +'/' + ComSim::IOTypeToStr(ioType);
        if( m_store.end() != m_store.find(interface) ) { continue; }

        ComSim* newComSim;
        if(ioType == ComSim::Slave) { newComSim = new SlaveComSim(boardName, *this); }
        else if(ioType == ComSim::Master) { newComSim = new MasterComSim(boardName, *this); }

        ROS_INFO_STREAM("find " << interface);
        m_store.emplace( interface, newComSim );
    }
}


