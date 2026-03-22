//INFO: for ALNS

#pragma once

#include "../DARPMD_ProblemInstance.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"

#include <ilcplex/ilocplex.h>

#include <map>

class SetCoveringSolver {
public:
    SetCoveringSolver(
        const DARPMD_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator
    );
    ~SetCoveringSolver();

    ALNSSolution solve(const std::map<int, std::vector<ALNSRoute>>& routePool);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;

    IloEnv env;

    // Map to quickly find which routes cover each request (built from routePool)
    std::map<int, int> requestToIndex;

    // As set covering can produce solutions with duplicates, we need a repair function to clean them up
    void repairSolution(ALNSSolution& sol);
    void warnIfDuplicateRequests(const ALNSSolution& sol) const;
};
