#pragma once

#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "DARPMD_ProblemInstance.h"

class ALNSEvaluator {
public:
    ALNSEvaluator(
        const DARPMD_ProblemInstance& data,
        const ALNSParams& params
    );

    void evaluateRoute(ALNSRoute& route);
    void evaluateSolution(ALNSSolution& sol);

    // This methods only priorize time window feasibility
    void evaluateRouteGreedy(ALNSRoute& route);
    void evaluateSolutionGreedy(ALNSSolution& sol);

    bool solutionHasViolations(const ALNSSolution&) const;

    double calculateExactDelta(const ALNSRoute& route, int requestId, int i, int j);
    double calculateGreedyDelta(const ALNSRoute& route, int requestId, int i, int j);
    double calculateGreedyDelta_2(const ALNSRoute& route, int requestId, int i, int j);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
};
