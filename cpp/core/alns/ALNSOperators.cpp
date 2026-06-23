#include "ALNSOperators.h"

#include "../MDDARP_ProblemInstance.h"
#include "ALNSEvaluator.h"
        
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

ALNSOperators::ALNSOperators(const MDDARP_ProblemInstance& instance,
                             const ALNSParams& parameters,
                             ALNSEvaluator& evaluator,
                             std::mt19937& randomEngine,
                             bool enableGICE,
                             bool enableNR
                            )
    : data(instance), params(parameters), evaluator(evaluator), rng(randomEngine) {
     
    // Set insertion method based on flags
    insertionMethod = (enableGICE) ? GICE : FTSE;
    
    // Set reduction method based on flags
    reductionMethod = (enableNR) ? REDUCTION : NONE;
}

void ALNSOperators::destroyRandom(ALNSSolution& sol, int q) {
    int n = data.N_requests;
    int removals = std::min(q, n);
    if (removals <= 0) return;

    std::vector<int> reqToRoute(n + 1, -1);

    for (size_t v = 0; v < sol.routes.size(); ++v) {
        const auto& seq = sol.routes[v].sequence;
        for (size_t i = 1; i < seq.size() - 1; ++i) { // skip depots
            int node = seq[i];
            if (data.isPickup(node)) {
                reqToRoute[node] = (int)v;
            }
        }
    }

    std::vector<int> requests(n);
    std::iota(requests.begin(), requests.end(), 1);

    // Fisher-Yates shuffle for the first removals elements
    for (int i = 0; i < removals; ++i) {
        std::uniform_int_distribution<int> dist(i, n - 1);
        int j = dist(rng);
        std::swap(requests[i], requests[j]);
    }

    std::unordered_set<int> nodesToRemove;
    nodesToRemove.reserve(removals * 2);
    
    std::vector<bool> routeNeedsUpdate(sol.routes.size(), false);

    for (int i = 0; i < removals; ++i) {
        int reqId = requests[i];
        int vIdx = reqToRoute[reqId];

        nodesToRemove.insert(reqId);                    // pickup
        nodesToRemove.insert(reqId + data.N_requests);  // delivery

        sol.unassignedRequests.insert(reqId);
        routeNeedsUpdate[vIdx] = true;
    }

    // Apply removals in a single pass for each affected route to optimize performance
    for (size_t v = 0; v < sol.routes.size(); ++v) {
        if (routeNeedsUpdate[v]) {
            auto& route = sol.routes[v];

            std::erase_if(route.sequence,
                [&nodesToRemove](int node) {
                    return nodesToRemove.contains(node);
                });
        }
    }

    evaluator.evaluateSolution(sol);
}

void ALNSOperators::destroyWorst(ALNSSolution& sol, int q) {
    std::vector<std::pair<double, int>> savingsMap; // <Saving, RequestID>

    // Calculate the cost of each request in the current solution
    for (size_t v = 0; v < sol.routes.size(); ++v) {
        ALNSRoute& route = sol.routes[v];
        if (route.sequence.size() <= 2) continue; // Only depot

        double currentCost = route.totalCost;

        // Identify requests in this route
        std::vector<int> requestsInRoute;
        for (int node : route.sequence) {
            if (data.isPickup(node)) {
                requestsInRoute.push_back(node);
            }
        }

        // Try removing each one to see how much we save
        ALNSRoute tempRoute = route;
        for (int reqId : requestsInRoute) {
            tempRoute.sequence.clear();
            std::vector<int> newSeq;
            int deliveryId = reqId + data.N_requests;

            for (int node : route.sequence) {
                if (node != reqId && node != deliveryId) newSeq.push_back(node);
            }
            tempRoute.sequence = newSeq;
            evaluator.evaluateRoute(tempRoute);

            double saving = currentCost - tempRoute.totalCost;
            savingsMap.push_back({saving, reqId});
        }
    }

    // Sort by greatest saving (Descending)
    std::sort(savingsMap.rbegin(), savingsMap.rend()); 

    // Remove the top 'q' (with a random factor to avoid pure determinism)
    // The parameter 'p' controls randomness (e.g., p=3)
    int removedCount = 0;
    while (removedCount < q && !savingsMap.empty()) {
        // Select based on biased index (to not always choose the strict #1)
        // index = floor(|L| * rand^p)
        double r = std::generate_canonical<double, 10>(rng);
        int idx = std::floor(savingsMap.size() * std::pow(r, params.worstRemovalPower)); // Assumes power ~ 3-6
        
        if (idx >= (int)savingsMap.size()) idx = (int)savingsMap.size() - 1;

        int reqToRemove = savingsMap[idx].second;
        
        // Physically remove from the solution
        for (auto& route : sol.routes) {
            auto& seq = route.sequence;
            auto itP = std::find(seq.begin(), seq.end(), reqToRemove);
            if (itP != seq.end()) {
                // Rebuild vector without P and D
                int deliveryId = reqToRemove + data.N_requests;
                std::vector<int> cleanSeq;
                for(int node : seq) {
                    if (node != reqToRemove && node != deliveryId) cleanSeq.push_back(node);
                }
                route.sequence = cleanSeq;
                break; // We found it in this route
            }
        }
        
        sol.unassignedRequests.insert(reqToRemove);
        savingsMap.erase(savingsMap.begin() + idx);
        removedCount++;
    }

    evaluator.evaluateSolution(sol);
}

void ALNSOperators::destroyShaw(ALNSSolution& sol, int q) {
    // Choose a random seed request that is currently assigned
    const std::vector<int>& assigned = data.P;
    std::uniform_int_distribution<> dis(0, assigned.size() - 1);
    int seedRequest = assigned[dis(rng)];

    std::vector<int> toRemove;
    toRemove.push_back(seedRequest);

    // Sort the rest of the requests by similarity to the seed
    std::vector<std::pair<double, int>> relatedList;
    for (int other : assigned) {
        if (other == seedRequest) continue;
        double R = calculateRelatedness(seedRequest, other);
        relatedList.push_back({R, other});
    }
    std::sort(relatedList.begin(), relatedList.end()); // Lower R is better

    // Select q-1 closest neighbors (with some randomness)
    while ((int)toRemove.size() < q && !relatedList.empty()) {
        double r = std::generate_canonical<double, 10>(rng);
        int idx = std::floor(relatedList.size() * std::pow(r, 6.0)); // Strong bias towards the beginning
        if (idx >= (int)relatedList.size()) idx = (int)relatedList.size() - 1;

        toRemove.push_back(relatedList[idx].second);
        relatedList.erase(relatedList.begin() + idx);
    }

    // Execute removal
    for (int req : toRemove) {
        for (auto& route : sol.routes) {
            // ... Same physical removal code as in destroyRandom/Worst ...
            std::vector<int> newSeq;
            int devId = req + data.N_requests;
            bool found = false;
            for (int n : route.sequence) {
                if (n == req || n == devId) found = true;
                else newSeq.push_back(n);
            }
            if (found) {
                route.sequence = newSeq;
                break;
            }
        }
        sol.unassignedRequests.insert(req);
    }
    
    evaluator.evaluateSolution(sol);
}

void ALNSOperators::repairGreedy(ALNSSolution& sol) {    
    std::vector<int> todo(sol.unassignedRequests.begin(), sol.unassignedRequests.end());
    sol.unassignedRequests.clear(); // Will re-add failures later
    std::shuffle(todo.begin(), todo.end(), rng);

    for (int reqId : todo) {
        int deliveryId = reqId + data.N_requests;
        double bestCostIncrease = std::numeric_limits<double>::max();
        int bestVehicle = -1;
        int bestPIdx = -1;
        int bestDIdx = -1;

        // Try all vehicles
        for (size_t v = 0; v < sol.routes.size(); ++v) {
            LocalInsertion insertion = findBestInsertion(insertionMethod, sol.routes[v], reqId);

            if (insertion.deltaCost < bestCostIncrease) {
                bestCostIncrease = insertion.deltaCost;
                bestVehicle = (int)v;
                bestPIdx = insertion.pIdx;
                bestDIdx = insertion.dIdx;
            }
        }

        auto& r = sol.routes[bestVehicle];
        r.sequence.insert(r.sequence.begin() + bestPIdx, reqId);
        r.sequence.insert(r.sequence.begin() + bestDIdx + 1, deliveryId);

        evaluator.evaluateRoute(r);
    }
}

void ALNSOperators::repairRegret2(ALNSSolution& sol) {
    int numReqs = data.N_requests;
    int numVehicles = sol.routes.size();

    // Initialize the cache
    insertionCache.assign(numReqs + 1, std::vector<LocalInsertion>(numVehicles));

    for (int reqId : sol.unassignedRequests) {
        for (int v = 0; v < numVehicles; ++v) {
            LocalInsertion ins = findBestInsertion(insertionMethod, sol.routes[v], reqId);
            insertionCache[reqId][v] = {ins.pIdx, ins.dIdx, ins.deltaCost};
        }
    }

    while (!sol.unassignedRequests.empty()) {
        int bestReqId = -1;
        double maxRegretValue = -1.0;
        LocalInsertion winMove;
        int winVehicle = -1;

        // Search for the request with the highest regret value
        for (int reqId : sol.unassignedRequests) {
            double bestCost = std::numeric_limits<double>::infinity();
            double secondBestCost = std::numeric_limits<double>::infinity();
            int bestVForThisReq = -1;
            LocalInsertion bestMoveForThisReq;

            for (int v = 0; v < numVehicles; ++v) {
                double cost = insertionCache[reqId][v].deltaCost;
                if (cost < bestCost) {
                    secondBestCost = bestCost;
                    bestCost = cost;
                    bestVForThisReq = v;
                    bestMoveForThisReq = insertionCache[reqId][v];
                } else if (cost < secondBestCost) {
                    secondBestCost = cost;
                }
            }

            double regret = (secondBestCost == std::numeric_limits<double>::infinity()) 
                            ? 100000.0 : (secondBestCost - bestCost);

            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                winVehicle = bestVForThisReq;
                winMove = bestMoveForThisReq;
            }
        }

        auto& r = sol.routes[winVehicle];
        r.sequence.insert(r.sequence.begin() + winMove.pIdx, bestReqId);
        r.sequence.insert(r.sequence.begin() + winMove.dIdx + 1, bestReqId + data.N_requests);
        evaluator.evaluateRoute(r);
        
        sol.unassignedRequests.erase(bestReqId);

        // Just update the cache for the affected vehicle
        for (int reqId : sol.unassignedRequests) {
            LocalInsertion ins = findBestInsertion(insertionMethod, sol.routes[winVehicle], reqId);
            insertionCache[reqId][winVehicle] = {ins.pIdx, ins.dIdx, ins.deltaCost};
        }
    }
}

double ALNSOperators::calculateRelatedness(int i, int j) {
    // Heuristic weights
    double w_dist = params.shawDistWeight;
    double w_time = params.shawTimeWeight;
    double w_demand = params.shawDemandWeight;

    double dist = data.getTravelTime(i, j); 
    
    // Temporal time difference: we can use the midpoint of the time windows as a representative time for each request
    double mu_i = (data.getTimeWindowStart(i) + data.getTimeWindowEnd(i)) / 2.0;
    double mu_j = (data.getTimeWindowStart(j) + data.getTimeWindowEnd(j)) / 2.0;
    double timeDiff = std::abs(mu_i - mu_j);
    
    // Demand difference
    double demandDiff = std::abs(data.getDemand(i) - data.getDemand(j));

    // Relatedness value
    return w_dist * dist + w_time * timeDiff + w_demand * demandDiff;
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertion(
        ALNSOperators::InsertionMethod method,
        const ALNSRoute& route,
        int reqId
    ) {
    LocalInsertion best;
    
    if (reductionMethod == REDUCTION) {
        switch (method) {
            case FTSE:
                return findBestInsertionExact_R(route, reqId);
            default: //case GICE:
                return findBestInsertionGreedy_R(route, reqId);
        }
    }
    else {
        switch (method) {
            case FTSE:
                return findBestInsertionExact(route, reqId);
            default: //case GICE:
                return findBestInsertionGreedy(route, reqId);
        }
    }
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertionExact(const ALNSRoute& route, int reqId) {
    LocalInsertion best;
    int n = route.sequence.size();

    ALNSRoute temp = route;

    for (int i = 1; i < n; ++i) { // Pickup can be inserted between any two nodes
        for (int j = i; j < n; ++j) { // Delivery must come after pickup
            double delta = evaluator.calculateExactDelta(route, temp, reqId, i, j);
            if (delta < best.deltaCost) {
                best.deltaCost = delta;
                best.pIdx = i;
                best.dIdx = j;
            }
        }
    }
    return best;
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertionGreedy(const ALNSRoute& route, int reqId) {
    ALNSOperators::LocalInsertion best;
    int n = route.sequence.size();

    for (int i = 1; i < n; ++i) { // Pickup can be inserted between any two nodes
        for (int j = i; j < n; ++j) { // Delivery must come after pickup
            double delta = evaluator.calculateGreedyDelta(route, reqId, i, j, best.deltaCost);
            if (delta < best.deltaCost) {
                best.deltaCost = delta;
                best.pIdx = i;
                best.dIdx = j;
            }
        }
    }
    return best;
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertionExact_R(const ALNSRoute& route, int reqId) {
    LocalInsertion best;
    ALNSRoute temp = route;
    int n = route.sequence.size();
    
    // 1. Determine which vertex is critical. According to Cordeau & Laporte, a vertex is critical if e_i != 0 or l_i != T.
    bool pickupIsCritical = data.getTimeWindowStart(reqId) > 0 || 
                            data.getTimeWindowEnd(reqId) < data.getVehicleMaxRouteTime(route.vehicleId);

    if (pickupIsCritical) {
        // PHASE 1: Find the best position for Pickup (i)
        int bestI = -1;
        double bestDeltaI = std::numeric_limits<double>::infinity();

        for (int i = 1; i < n; ++i) {
            // We evaluate the insertion by placing the delivery right after the pickup (j = i)
            double delta = evaluator.calculateExactDelta(route, temp, reqId, i, i);
            if (delta < bestDeltaI) {
                bestDeltaI = delta;
                bestI = i;
            }
        }

        // PHASE 2: Maintaining the Pickup fixed (bestI), test all valid positions for the Delivery (j)
        if (bestI != -1) {
            for (int j = bestI; j < n; ++j) {
                double delta = evaluator.calculateExactDelta(route, temp, reqId, bestI, j);
                if (delta < best.deltaCost) {
                    best.deltaCost = delta;
                    best.pIdx = bestI;
                    best.dIdx = j;
                }
            }
        }
    } else {
        // PHASE 1: The Delivery is the critical vertex. Find the best position (j)
        int bestJ = -1;
        double bestDeltaJ = std::numeric_limits<double>::infinity();

        for (int j = 1; j < n; ++j) {
            // We evaluate the insertion by placing the pickup right before the delivery (i = j)
            double delta = evaluator.calculateExactDelta(route, temp, reqId, j, j);
            if (delta < bestDeltaJ) {
                bestDeltaJ = delta;
                bestJ = j;
            }
        }

        // PHASE 2: Maintaining the Delivery fixed (bestJ), test all valid positions for the Pickup (i)
        if (bestJ != -1) {
            for (int i = 1; i <= bestJ; ++i) {
                double delta = evaluator.calculateExactDelta(route, temp, reqId, i, bestJ);
                if (delta < best.deltaCost) {
                    best.deltaCost = delta;
                    best.pIdx = i;
                    best.dIdx = bestJ;
                }
            }
        }
    }
    return best;
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertionGreedy_R(const ALNSRoute& route, int reqId) {
    LocalInsertion best;
    int n = route.sequence.size();

    // 1. Determine which vertex is critical. According to Cordeau & Laporte, a vertex is critical if e_i != 0 or l_i != T.
    bool pickupIsCritical = data.getTimeWindowStart(reqId) > 0 ||
                            data.getTimeWindowEnd(reqId) < data.getVehicleMaxRouteTime(route.vehicleId);

    if (pickupIsCritical) {
        // PHASE 1: Find the best position for Pickup (i)
        int bestI = -1;
        double bestDeltaI = std::numeric_limits<double>::infinity();

        for (int i = 1; i < n; ++i) {
            // We evaluate the insertion by placing the delivery right after the pickup (j = i)
            double delta = evaluator.calculateGreedyDelta(route, reqId, i, i, bestDeltaI);
            if (delta < bestDeltaI) {
                bestDeltaI = delta;
                bestI = i;
            }
        }

        // PHASE 2: Maintaining the Pickup fixed (bestI), test all valid positions for the Delivery (j)
        if (bestI != -1) {
            for (int j = bestI; j < n; ++j) {
                double delta = evaluator.calculateGreedyDelta(route, reqId, bestI, j, best.deltaCost);
                if (delta < best.deltaCost) {
                    best.deltaCost = delta;
                    best.pIdx = bestI;
                    best.dIdx = j;
                }
            }
        }
    } else {
        // PHASE 1: The Delivery is the critical vertex. Find the best position (j)
        int bestJ = -1;
        double bestDeltaJ = std::numeric_limits<double>::infinity();

        for (int j = 1; j < n; ++j) {
            // We evaluate the insertion by placing the pickup right before the delivery (i = j)
            double delta = evaluator.calculateGreedyDelta(route, reqId, j, j, bestDeltaJ);
            if (delta < bestDeltaJ) {
                bestDeltaJ = delta;
                bestJ = j;
            }
        }

        // PHASE 2: Maintaining the Delivery fixed (bestJ), test all valid positions for the Pickup (i)
        if (bestJ != -1) {
            for (int i = 1; i <= bestJ; ++i) {
                double delta = evaluator.calculateGreedyDelta(route, reqId, i, bestJ, best.deltaCost);
                if (delta < best.deltaCost) {
                    best.deltaCost = delta;
                    best.pIdx = i;
                    best.dIdx = bestJ;
                }
            }
        }
    }
    return best;
}