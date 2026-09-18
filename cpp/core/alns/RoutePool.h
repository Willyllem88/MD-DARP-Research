#pragma once

#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <cstddef>
#include <limits>

#include "ALNSRoute.h"
#include "ALNSParams.h"
#include "../MDDARP_ProblemInstance.h"

class RoutePool {
public:
    RoutePool(const MDDARP_ProblemInstance& instance, const ALNSParams& params);
    ~RoutePool() = default;

    // Add the route if it's not a duplicate and if it has potential to improve the current best solution
    void addRoute(
        const ALNSRoute& route,
        double currentBestTotalSolutionCost = std::numeric_limits<double>::infinity()
    );

    // Get all routes (useful for the solver to read them)
    const std::unordered_map<int, std::vector<ALNSRoute>>& getRoutes();

    int getTotalNumberOfRoutes() const;
    
    void prune(double currentBestTotalSolutionCost, bool pruneSCP = false);

    // Clear the entire pool
    void clear();

private:
    // Functions wich return the lower bound for pruning columns
    double calculateLowerBound(const ALNSRoute& route, int k);
    double calculateLowerBoundValid(const ALNSRoute& route, int k);
    double calculateLowerBoundHeuristic(const ALNSRoute& route, int k, double xi);

    const MDDARP_ProblemInstance& problemInstance;
    const ALNSParams& params;

    // Key: VehicleID, Value: List of routes
    std::unordered_map<int, std::vector<ALNSRoute>> routePool;

    struct VectorHash {
        size_t operator()(const std::vector<int>& v) const {
            size_t h = 0;
            for (int x : v) {
                h = h * 31 + std::hash<int>{}(x);
            }
            return h;
        }
    };
    using NodeSetKey = std::vector<int>;

    // For each vehicle, store the best route for each unique set of nodes (ignoring order)
    std::unordered_map<int, 
        std::unordered_map<NodeSetKey, ALNSRoute, VectorHash>
    > bestRoutes;

    // For Lower-Bound pruning, empty meaning min cost route for a vehicle.
    std::vector<double> emptyRouteCost; // Cost of an empty route for i-th vehicle.
    double emptySolutionCost = 0.0;     // Cost of an empty solution
    std::vector<std::vector<double>> lambda_ik; // Min cost of servicing i-th request if
                                                // k-th vehicle is unavailable.
};