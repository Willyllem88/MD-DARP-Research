#include "RoutePool.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

void RoutePool::addRoute(const ALNSRoute& route, double currentBestTotalSolutionCost) {
    if (route.sequence.empty()) return;
    if (route.totalCost > currentBestTotalSolutionCost) return;

    // Canonical key (order-independent)
    NodeSetKey key = route.sequence;
    std::sort(key.begin(), key.end());

    auto& vehicleMap = bestRoutes[route.vehicleId];
    auto it = vehicleMap.find(key);

    if (it == vehicleMap.end()) {
        vehicleMap[key] = route;
    } else {
        if (route.totalCost < it->second.totalCost) {
            it->second = route;
        } else {
            return; // if worst, ignore
        }
    }
}

const std::unordered_map<int, std::vector<ALNSRoute>>& RoutePool::getRoutes() {
    routePool.clear();

    for (auto& [vehicleId, mapRoutes] : bestRoutes) {
        auto& vec = routePool[vehicleId];
        vec.reserve(mapRoutes.size());

        for (auto& [_, route] : mapRoutes) {
            vec.push_back(route);
        }
    }

    return routePool;
}

void RoutePool::clear() {
    routePool.clear();
    bestRoutes.clear();
}