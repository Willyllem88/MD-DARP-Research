#pragma once

#include "MDDARP_ProblemInstance.h"
#include "MDDARP_ResultInstance.h"
#include "logger.h"

class Solver {
public:
    explicit Solver(bool verbose = false) : 
        logger(verbose), verbose(verbose) {}

    virtual ~Solver() = default;

    virtual void solve() = 0;

    virtual MDDARP_ResultInstance getResult() const = 0;

    virtual std::string name() const = 0;

protected:
    Logger logger;
    bool verbose = false;
};
