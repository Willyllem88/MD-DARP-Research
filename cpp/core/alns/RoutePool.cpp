#include "RoutePool.h"

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>

RoutePool::RoutePool(const DARPMD_ProblemInstance& instance) : problemInstance(instance) {

    int numNodes = problemInstance.max_node_id + 1;
    int numVehicles = problemInstance.K_vehicles;

    emptyRouteCosts.resize(numVehicles+1, 0.0);
    totalEmptyCost = 0.0;
    
    // Lambda to run Dijkstra's algorithm for a given vehicle's start and end nodes
    auto runDijkstra = [&](int start, int end, int vehicleId) -> std::pair<double, std::vector<int>> {
        std::vector<double> dist(numNodes, std::numeric_limits<double>::infinity());
        std::vector<int> parent(numNodes, -1);
        
        // Priority queue stores {accumulated_distance, current_node}
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

        dist[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[u]) continue;
            if (u == end) break;

            for (int v = 1; v < numNodes; ++v) {
                if (u == v) continue;
                
                double weight = problemInstance.getCost(u, v, vehicleId); 
                
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // Reconstruct path from end to start using parent array
        std::vector<int> path;
        if (dist[end] != std::numeric_limits<double>::infinity()) {
            for (int at = end; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end()); // Invert for correct order
        } else {
            // Security fallback (in case the graph is not fully connected)
            path = {start, end}; 
        }

        return {dist[end], path};
    };

    // Initialize empty routes for each vehicle and calculate their costs using Dijkstra's algorithm
    for (int k = 1; k <= numVehicles; ++k) {
        int startNode = problemInstance.getVehicleStartNode(k);
        int endNode = problemInstance.getVehicleEndNode(k);

        auto [minCost, path] = runDijkstra(startNode, endNode, k);

        emptyRouteCosts[k] = minCost;
        totalEmptyCost += minCost;

        ALNSRoute emptyRoute;
        emptyRoute.vehicleId = k;
        emptyRoute.sequence = path;
        emptyRoute.totalCost = minCost;
        emptyRoute.distanceCost = minCost; 
        emptyRoute.isFeasible = true; 

        addRoute(emptyRoute, std::numeric_limits<double>::infinity());
    }
}

void RoutePool::addRoute(const ALNSRoute& route, double currentBestTotalSolutionCost) {
    if (route.sequence.empty()) return;

    double lowerBound = route.totalCost;
    if (!emptyRouteCosts.empty() && route.vehicleId < (int)emptyRouteCosts.size())
        lowerBound += (totalEmptyCost - emptyRouteCosts[route.vehicleId]);

    if (lowerBound >= currentBestTotalSolutionCost) return;

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

    emptyRouteCosts.clear();
    totalEmptyCost = 0.0;
}

void RoutePool::prune(double currentBestTotalSolutionCost, bool pruneSCP) {
    for (auto& [vehicleId, mapRoutes] : bestRoutes) {
        for (auto it = mapRoutes.begin(); it != mapRoutes.end(); ) {
            double lowerBound = it->second.totalCost;
            if (!emptyRouteCosts.empty() && vehicleId < (int)emptyRouteCosts.size())
                lowerBound += (totalEmptyCost - emptyRouteCosts[vehicleId]);
            
            if (lowerBound > currentBestTotalSolutionCost)
                it = mapRoutes.erase(it);
            else
                ++it;
        }

        if (pruneSCP) {
            for (auto itA = mapRoutes.begin(); itA != mapRoutes.end(); ) {
                bool dominated = false;
                
                for (auto itB = mapRoutes.begin(); itB != mapRoutes.end(); ++itB) {
                    if (itA == itB) continue;

                    // Comprobación de dominancia:
                    // Si la Ruta B es más barata o cuesta igual que la Ruta A...
                    if (itB->second.totalCost <= itA->second.totalCost) {
                        // ... y la Ruta B cubre todo lo que cubre la Ruta A.
                        // std::includes funciona perfectamente porque it->first (NodeSetKey) ya está ordenado.
                        if (std::includes(itB->first.begin(), itB->first.end(),
                                        itA->first.begin(), itA->first.end())) {
                            dominated = true;
                            break;
                        }
                    }
                }

                if (dominated) {
                    itA = mapRoutes.erase(itA); // A es dominada, la borramos
                } else {
                    ++itA; // A sobrevive
                }
            }
        }
    }
}