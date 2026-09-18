#include "RoutePool.h"

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>

RoutePool::RoutePool(const MDDARP_ProblemInstance& instance, const ALNSParams& params)
    : problemInstance(instance), params(params) {

    int numNodes = problemInstance.max_node_id + 1;
    int numReqs = problemInstance.N_requests;
    int numVeh = problemInstance.K_vehicles;

    emptyRouteCost.assign(numVeh+1, 0.0);
    emptySolutionCost = 0.0;
    lambda_ik.assign(numReqs+1, std::vector<double>(numVeh+1));
    
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

    // Initialize empty routes for each vehicle and calculate their costs using Dijkstra's algorithm,
    for (int k = 1; k <= numVeh; ++k) {
        int startNode = problemInstance.getVehicleStartNode(k);
        int endNode = problemInstance.getVehicleEndNode(k);

        auto [minVehCost, path] = runDijkstra(startNode, endNode, k);

        emptyRouteCost[k] = minVehCost;
        emptySolutionCost += minVehCost;
    }

    // Initialize lambda_ik, marginal_ik is an auxiliary matrix to do so
    std::vector<std::vector<double>> marginal(numReqs + 1, std::vector<double>(numVeh + 1, 0.0));

    for (int k = 1; k <= numVeh; ++k) {
        int startNode = problemInstance.getVehicleStartNode(k);
        int endNode = problemInstance.getVehicleEndNode(k);

        for (int i = 1; i <= numReqs; ++i) {
            int deliveryNode = numReqs + i;
            double routeWithICost = runDijkstra(startNode, i, k).first 
                                + runDijkstra(i, deliveryNode, k).first 
                                + runDijkstra(deliveryNode, endNode, k).first;

            marginal[i][k] = routeWithICost - emptyRouteCost[k];
        }
    }
    for (int i = 1; i <= numReqs; ++i) {
        for (int k = 1; k <= numVeh; ++k) {
            double minMarginal = std::numeric_limits<double>::infinity();
            for (int kp = 1; kp <= numVeh; ++kp) {
                if (kp == k) continue; // k' != k
                minMarginal = std::min(minMarginal, marginal[i][kp]);
            }
            lambda_ik[i][k] = minMarginal;
        }
    }

}

void RoutePool::addRoute(const ALNSRoute& route, double currentBestTotalSolutionCost) {
    if (route.sequence.empty()) return;
    if (route.isFeasible == false) return;

    double lowerBound = calculateLowerBound(route, route.vehicleId);

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

int RoutePool::getTotalNumberOfRoutes() const {
    int total = 0;
    for (const auto& [vehicleId, mapRoutes] : bestRoutes) {
        total += mapRoutes.size();
    }
    return total;
}

void RoutePool::clear() {
    routePool.clear();
    bestRoutes.clear();

    emptyRouteCost.clear();
    emptySolutionCost = 0.0;
    lambda_ik.clear();
}

void RoutePool::prune(double currentBestTotalSolutionCost, bool pruneSCP) {
    for (auto& [vehicleId, mapRoutes] : bestRoutes) {
        for (auto it = mapRoutes.begin(); it != mapRoutes.end(); ) {
            double lowerBound = calculateLowerBound(it->second, vehicleId);
            if (lowerBound > currentBestTotalSolutionCost)
                it = mapRoutes.erase(it);
            else
                ++it;
        }

        // TODO: could be in a separatted function
        if (pruneSCP) {
            for (auto itA = mapRoutes.begin(); itA != mapRoutes.end(); ) {
                bool dominated = false;
                
                for (auto itB = mapRoutes.begin(); itB != mapRoutes.end(); ++itB) {
                    if (itA == itB) continue;

                    // Dominance check: 
                    // if B is cheaper or equal cost than A, and B covers all nodes that 
                    // A covers, then A is dominated by B.
                    if (itB->second.totalCost <= itA->second.totalCost) {
                        // ... and if route B covers all the nodes that route A covers.
                        // std::includes works perfectly because it->first (NodeSetKey) is already sorted.
                        if (std::includes(itB->first.begin(), itB->first.end(),
                                        itA->first.begin(), itA->first.end())) {
                            dominated = true;
                            break;
                        }
                    }
                }

                if (dominated) {
                    itA = mapRoutes.erase(itA); // A is dominated, we remove it
                } else {
                    ++itA; // A survives, we keep it
                }
            }
        }
    }
}

double RoutePool::calculateLowerBound(const ALNSRoute& route, int k){
    if (params.lowerBound_xi < 0) return calculateLowerBoundValid(route, k);
    else return calculateLowerBoundHeuristic(route, k, params.lowerBound_xi);
}

double RoutePool::calculateLowerBoundValid(const ALNSRoute& route, int k) {
    double max_lambda_ik = 0.0;
    for (int i = 1; i <= problemInstance.N_requests; ++i) {
        if (route.containsNode(i)) continue;
        max_lambda_ik = std::max(max_lambda_ik, lambda_ik[i][k]);
    }

    return route.totalCost + (emptySolutionCost - emptyRouteCost[k]) + max_lambda_ik;
}

double RoutePool::calculateLowerBoundHeuristic(const ALNSRoute& route, int k, double xi) {
    int notRsize = problemInstance.N_requests - (route.getRouteSize() - 2) / 2;
    int p = std::max(1, static_cast<int>(std::floor(xi * notRsize)));
    double sum_max_p = 0.0;

    std::priority_queue<double> topK;
    for (int i = 1; i <= problemInstance.N_requests; i++) {
        if (route.containsNode(i)) continue;
        topK.push(lambda_ik[i][k]);
    }
    for (int i = 0; i < p; ++i) {
        if (topK.empty()) break; //Should not happen
        sum_max_p += topK.top();
        topK.pop();
    }
    return route.totalCost + (emptySolutionCost - emptyRouteCost[k]) + sum_max_p;
}