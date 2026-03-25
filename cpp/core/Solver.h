#pragma once

#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"
#include "logger.h"

class Solver {
public:
    explicit Solver(bool verbose = false) : 
        logger(verbose), verbose(verbose) {}

    virtual ~Solver() = default;

    virtual void solve() = 0;

    virtual DARPMD_ResultInstance getResult() const = 0;

    virtual std::string name() const = 0;

protected:
    Logger logger;
    bool verbose = false;
};
