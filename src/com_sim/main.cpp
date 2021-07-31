#include <ros/ros.h>

#include "common/TopicMonitor.h"
#include "com_sim/ComSimObserver.h"


const std::string &nodename = "com_sim";


int main(int argc, char** argv)
{
    ros::init(argc, argv, nodename);
    
    ComSimObserver comSimObserver;
    auto checkPubTopics = std::bind(&ComSimObserver::checkPublishedTopics, &comSimObserver, std::placeholders::_1);

    TopicMonitor topicMonitor;
    topicMonitor.onPublishedTopics(checkPubTopics);

    ros::spin();

    return 0;
}