#include "com_sim/SlaveChecker.h"

#include <cmath>


bool SlaveChecker::check(const ComSim& from, const ComSim& to) 
{
    if(to.getIOType() == ComSim::IOType::Slave) { return false; }
    else if(to.getIOType() == ComSim::IOType::Master) 
    {
        auto fromCoord = from.getCoordinates();
        auto toCoord = to.getCoordinates();

        double fromHeight = fromCoord->pose.position.z;
        double toHeight = toCoord->pose.position.z;

        //slave height should be lesser then master height 
        if(fromHeight >= toHeight) { return false; }

        double deltaHeight = toHeight - fromHeight;

        //TODO - hardcode! Angle should be dynamicly set
        double radius = tan(45)*toHeight;
        double subRadius = (radius*deltaHeight)/toHeight;

        double distance = evalDistance(from, to);

        return subRadius >= distance;
    }
    
    return false;
}
