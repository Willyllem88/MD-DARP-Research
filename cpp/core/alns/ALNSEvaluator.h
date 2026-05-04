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

    // This methods only priorize time window feasibility (NOT USED)
    void evaluateRouteGreedy(ALNSRoute& route);
    void evaluateSolutionGreedy(ALNSSolution& sol);

    // Different methods to calculate the cost delta of inserting a 
    // request at positions i (pickup) and j (delivery) in a given 
    // route, without modifying the original route

    // - Unoptimized but exact: simulates the full evaluation procedure O(n^2)
    double calculateExactDelta(const ALNSRoute& route, int requestId, int i, int j);

    // - Optimized but approximate: simulates a simplified evaluation O(n)
    double calculateGreedyDelta(const ALNSRoute& route, int requestId, int i, int j, double upper_bound);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
};
