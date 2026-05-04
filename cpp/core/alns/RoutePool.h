#pragma once

#include "ALNSRoute.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <cstddef>
#include <limits>

class RoutePool {
public:
    RoutePool() = default;
    ~RoutePool() = default;

    // Añade la ruta si no está duplicada
    void addRoute(
        const ALNSRoute& route,
        double currentBestTotalSolutionCost = std::numeric_limits<double>::infinity()
    );

    // Obtiene todas las rutas (útil para que el solver las lea)
    const std::unordered_map<int, std::vector<ALNSRoute>>& getRoutes();

    
    void prune(double currentBestTotalSolutionCost);

    // Limpia todo el pool
    void clear();

private:
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
    std::unordered_map<int, std::unordered_map<NodeSetKey, ALNSRoute, VectorHash>> bestRoutes;
};