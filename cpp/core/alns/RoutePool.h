#pragma once

#include "ALNSRoute.h"
#include "../DARPMD_ProblemInstance.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <cstddef>
#include <limits>

class RoutePool {
public:
    RoutePool(const DARPMD_ProblemInstance& instance);
    ~RoutePool() = default;

    // Add the route if it's not a duplicate and if it has potential to improve the current best solution
    void addRoute(
        const ALNSRoute& route,
        double currentBestTotalSolutionCost = std::numeric_limits<double>::infinity()
    );

    // Get all routes (useful for the solver to read them)
    const std::unordered_map<int, std::vector<ALNSRoute>>& getRoutes();

    
    void prune(double currentBestTotalSolutionCost, bool pruneSCP = false);

    // Clear the entire pool
    void clear();

private:
    const DARPMD_ProblemInstance& problemInstance;

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

    std::vector<double> emptyRouteCosts; 
    double totalEmptyCost = 0.0;
};