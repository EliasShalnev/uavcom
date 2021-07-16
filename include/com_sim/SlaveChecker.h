#pragma once

#include "com_sim/ComChecker.h"

class SlaveChecker : public ComChecker
{
public:
    SlaveChecker() = default;
    SlaveChecker(const SlaveChecker&) = delete;
    SlaveChecker& operator=(const SlaveChecker&) = delete;
    ~SlaveChecker() override = default;

    bool check(const ComSim& from, const ComSim& to) override;
};

