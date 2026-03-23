#pragma once

#include "../DARPMD_ProblemInstance.h"
#include "../logger.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"

#include <ilcplex/ilocplex.h>

#include <map>

class SetPartitioningSolver {
public:
    SetPartitioningSolver(
        const DARPMD_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator,
        Logger& logger
    );
    ~SetPartitioningSolver();

    ALNSSolution solve(const std::map<int, std::vector<ALNSRoute>>& routePool);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
    Logger& logger;
    IloEnv env;

    // Map to quickly find which routes cover each request (built from routePool)
    std::map<int, int> requestToIndex;
};
