#pragma once

#include "../DARPMD_ProblemInstance.h"
#include "../logger.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"
#include "SetBasedSolver.h"

#include <ilcplex/ilocplex.h>

#include <map>

class SetPartitioningSolver : public SetBasedSolver {
public:
    SetPartitioningSolver(
        const DARPMD_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator,
        Logger& logger
    );
    ~SetPartitioningSolver() {};

    bool solve(ALNSSolution& newSol, double maxTime) override;

private:
    // Map to quickly find which routes cover each request (built from routePool)
    std::map<int, int> requestToIndex;
};
