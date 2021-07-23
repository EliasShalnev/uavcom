#include "com_sim/ComSimObserver.h"

#include "common/TopicHelper.h"

#include "com_sim/ComSim.h"
#include "com_sim/SlaveComSim.h"
#include "com_sim/MasterComSim.h"

ComSimObserver::~ComSimObserver() 
{
    // for(auto uavComSimIt : m_store) { delete uavComSimIt.second; }
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
        TopicHelper topicHelper(topicName);
        ComSim::IOType ioType;

        if(topicHelper.size() != IO_TOPIC_SIZE) { continue; }

        //проверка топика на /cone_output
        auto segmentIt = --topicHelper.end();
        if(*segmentIt == '/'+def::g_cone+def::g_output ) { ioType = ComSim::IOType::Master; }
        else
        {
            //проверка топика на /output
            if(*segmentIt == '/'+def::g_output) { ioType = ComSim::IOType::Slave; }
            else { continue; }
        }

        //проверка топика на /uav_com
        segmentIt = --segmentIt;
        if(*segmentIt != '/'+def::g_uavNodeName) { continue; }

        //Если в префиксе нет /scout или /bomber - отбрасываем топик
        def::BoardName boardName = *topicHelper.begin();
        if(boardName.find(ComSim::scout) == std::string::npos &&
           boardName.find(ComSim::bomber) == std::string::npos)
        {
            ROS_ERROR_STREAM("Invalid boardname " << boardName);
            continue;
        }

        std::string interface = boardName +'/' + ComSim::IOTypeToStr(ioType);
        if( m_store.end() != m_store.find(interface) ) { continue; }

        ComSim* newComSim;
        if(ioType == ComSim::Slave) { newComSim = new SlaveComSim(boardName, *this); }
        else if(ioType == ComSim::Master) { newComSim = new MasterComSim(boardName, *this); }

        ROS_INFO_STREAM("find " << interface);
        m_store.emplace( interface, newComSim );
    }
}


