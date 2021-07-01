#include "uav_com/QtROSNode.h"


QtROSNode::QtROSNode(int argc, char *argv[], const std::string &nodeName, uint32_t options)
{
    ros::init(argc, argv, nodeName, options);
    m_nodeHandle = new ros::NodeHandle;
}

QtROSNode::~QtROSNode() { delete m_nodeHandle; }

void QtROSNode::run()
{
    ROS_INFO_STREAM("ROS spin thread " << QThread::currentThread());
    while( ros::ok() )
    {
        ros::spinOnce();
    }
    std::cout << "Exitting from ROS spin thread..." << std::endl;
}
