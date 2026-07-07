#pragma once

#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "MDDARP_ProblemInstance.h"

class ALNSEvaluator {
public:
    ALNSEvaluator(
        const MDDARP_ProblemInstance& data,
        const ALNSParams& params
    );

    // Evaluate and schedule a single route and update its metrics, uses the Forward Time Slack
    //  to minimize violations, as in Cordeau and Laporte (2003).
    void evaluateRoute(ALNSRoute& route);

    // Evaluate a route and return its total cost, without modifying the route object, it is not guaranteed
    // to be consistent with the route's internal metrics, as it does not update them. It uses a cache to
    // avoid redundant calculations. Must be carfully used, as it does not update the route's internal state.
    double calculateRouteCost(ALNSRoute& route);

    // Evaluate a full solution, including all routes and unassigned requests, uses the previous
    //  evaluateRoute function to evaluate each route.
    void evaluateSolution(ALNSSolution& sol);

    // Calculate the change in objective value if a request is inserted into a route at positions i and j
    double calculateDelta(const ALNSRoute& route, ALNSRoute& temp, int requestId, int i, int j);

private:
    const MDDARP_ProblemInstance& data;
    const ALNSParams& params;

    std::unordered_map<std::size_t, double> objectiveValCache; // Cache for objective values of routes
};
