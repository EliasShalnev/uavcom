#pragma once

#include <string>

#include <QThread>

#include <ros/ros.h>

class QtROSNode : public QThread
{
public:
    QtROSNode(int argc, char *argv[], const std::string &nodeName, uint32_t options = 0);
    virtual ~QtROSNode();

    ros::NodeHandle getNodeHandle() { return *m_nodeHandle; } //передается по значению, так как в сущности является shared_ptr

    virtual void run() override;
    
private:
    ros::NodeHandle* m_nodeHandle; //хранится по указателю из-за того, что его создание должно быть после ros::init(...)
};

