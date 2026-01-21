#pragma once

#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"

class Solver {
public:
    virtual ~Solver() = default;

    virtual void solve() = 0;

    virtual DARPMD_ResultInstance getResult() const = 0;

    virtual std::string name() const = 0;
};
