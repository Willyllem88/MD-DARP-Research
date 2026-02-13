#pragma once

class ALNSSolver; // Forward declaration to avoid circular dependency

#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "DARPMD_ProblemInstance.h"

class ALNSEvaluator {
public:
    //TODO: won't reference the solver, just for now to call addRouteToPool, we can refactor later to decouple
    ALNSEvaluator(
        ALNSSolver& solver,
        const DARPMD_ProblemInstance& data,
        const ALNSParams& params);

    void evaluateRoute(ALNSRoute& route);
    void evaluateSolution(ALNSSolution& sol);

    bool solutionHasViolations(const ALNSSolution&) const;

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
    ALNSSolver& solver;
};
