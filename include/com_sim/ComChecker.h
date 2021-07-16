#pragma once

#include "com_sim/ComSim.h"

class ComChecker
{
public:
    ComChecker() = default;
    ComChecker(const ComChecker&) = delete;
    ComChecker& operator=(const ComChecker&) = delete;
    virtual ~ComChecker() = default;

    virtual bool check(const ComSim& from, const ComSim& to) = 0;

    double evalDistance(const ComSim& from, const ComSim& to)
    {
        auto fromCoord = from.getCoordinates();
        auto toCoord = to.getCoordinates();     

        double fromX = fromCoord->pose.position.x;
        double toX = toCoord->pose.position.x;
        double deltaX = fromX-toX;

        double fromY = fromCoord->pose.position.y;
        double toY = toCoord->pose.position.y;
        double deltaY = fromY-toY;
        
        return std::sqrt( ( std::pow(deltaX, 2) + std::pow(deltaY, 2) ) );
    }
};


// double ComChecker::evalDistance(const ComSim& from, const ComSim& to) 


