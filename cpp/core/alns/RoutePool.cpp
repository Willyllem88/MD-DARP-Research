#include "RoutePool.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

void RoutePool::addRoute(const ALNSRoute& route) {
    // Ignore empty routes
    if (route.sequence.empty()) {
        return;
    }

    // Obtain the set of seen routes for this vehicle
    auto& vehicleSeenRoutes = seenRoutes[route.vehicleId];
    const auto [it, inserted] = vehicleSeenRoutes.insert(route.sequence);

    // If it's a new route for this vehicle, save it in the pool
    if (inserted) {
        routePool[route.vehicleId].push_back(route);
    }
}

void RoutePool::purgeColumns() {
    int totalRoutesBefore = 0;
    int totalRoutesAfter = 0;

    auto getHashNotOrderDependent = [](const std::vector<int>& seq) {
        size_t hash = 0;
        for (int nodeId : seq) {
            hash ^= std::hash<int>{}(nodeId);
        }
        return hash;
    };

    // We insert 
    for (auto& [vehicleId, routes] : routePool) {
        totalRoutesBefore += routes.size();

        std::unordered_map<size_t, ALNSRoute> bestRouteForNodeSet;

        for (const auto& route : routes) {
            size_t nodeSetHash = getHashNotOrderDependent(route.sequence);

            auto it = bestRouteForNodeSet.find(nodeSetHash);
            if (it == bestRouteForNodeSet.end()) {
                bestRouteForNodeSet[nodeSetHash] = route;
            } else {
                if (route.totalCost < it->second.totalCost) {
                    it->second = route;
                }
            }
        }

        // Reconstruct the route list with only the best route for each unique node set
        routes.clear();
        routes.reserve(bestRouteForNodeSet.size());
        for (const auto& [nodeSet, bestRoute] : bestRouteForNodeSet) {
            routes.push_back(bestRoute);
        }

        totalRoutesAfter += routes.size();
    }
}