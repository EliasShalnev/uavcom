#pragma once

#include "com_sim/SlaveChecker.h"

class MasterChecker : public SlaveChecker
{
public:
    MasterChecker() = default;
    MasterChecker(const MasterChecker&) = delete;
    MasterChecker& operator=(const MasterChecker&) = delete;
    ~MasterChecker() override = default;

    bool check(const ComSim& from, const ComSim& to) override;
};

