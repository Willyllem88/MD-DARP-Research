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

    // Different methods to calculate the cost delta of inserting a 
    // request at positions i (pickup) and j (delivery) in a given 
    // route, without modifying the original route

    // - Unoptimized but exact: simulates the full evaluation procedure O(n^2)
    double calculateExactDelta(const ALNSRoute& route, int requestId, int i, int j);

    // - Optimized but approximate: simulates a simplified evaluation O(n)
    double calculateDelta(const ALNSRoute& route, int requestId, int i, int j);

    // - Unoptimized and greedy: simulates a simplified evaluation that only considers time windows, O(n)
    double calculateGreedyDelta_2(const ALNSRoute& route, int requestId, int i, int j);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
};
