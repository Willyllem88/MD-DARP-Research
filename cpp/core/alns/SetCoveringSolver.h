//INFO: for ALNS

#pragma once

#include "../DARPMD_ProblemInstance.h"
#include "../logger.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"
#include "SetBasedSolver.h"

#include <ilcplex/ilocplex.h>

#include <map>

class SetCoveringSolver : public SetBasedSolver {
public:
    SetCoveringSolver(
        const DARPMD_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator,
        Logger& logger
    );
    ~SetCoveringSolver() {};

    bool solve(ALNSSolution& newSol) override;

private:
    // Map to quickly find which routes cover each request (built from routePool)
    std::map<int, int> requestToIndex;

    // As set covering can produce solutions with duplicates, we need a repair function to clean them up
    void repairSolution(ALNSSolution& sol);
    void warnIfDuplicateRequests(const ALNSSolution& sol) const;
    void removeRequestFromRoute(ALNSRoute& route, int reqId) const;
};
