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
                             bool enableNR
                            )
    : data(instance), params(parameters), evaluator(evaluator), rng(randomEngine) {
     
    // Set reduction method based on flags
    reductionMethod = (enableNR) ? REDUCTION : NONE;
}

void ALNSOperators::destroyRandom(ALNSSolution& sol, int q) {
    int n = data.N_requests;
    int removals = std::min(q, n);
    if (removals <= 0) return;

    std::vector<int> requests(n);
    std::iota(requests.begin(), requests.end(), 1);

    // Fisher-Yates shuffle for the first removals elements
    for (int i = 0; i < removals; ++i) {
        std::uniform_int_distribution<int> dist(i, n - 1);
        int j = dist(rng);
        std::swap(requests[i], requests[j]);
    }

    std::vector<int> toRemove(requests.begin(), requests.begin() + removals);

    sol.removeRequests(toRemove, n);
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
        int idx = std::floor(savingsMap.size() * std::pow(r, params.worstRemovalPower));
        if (idx >= (int)savingsMap.size()) idx = (int)savingsMap.size() - 1;

        int reqToRemove = savingsMap[idx].second;
        
        sol.removeRequest(reqToRemove, data.N_requests);
        
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
        double R = calculateRelatedness(seedRequest, other, sol);
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

    sol.removeRequests(toRemove, data.N_requests);
    evaluator.evaluateSolution(sol);
}

void ALNSOperators::repairGreedy(ALNSSolution& sol) {    
    std::vector<int> todo(sol.unassignedRequests.begin(), sol.unassignedRequests.end());
    std::shuffle(todo.begin(), todo.end(), rng);

    for (int reqId : todo) {
        double bestCostIncrease = std::numeric_limits<double>::max();
        int bestVehicle = -1;
        int bestPIdx = -1;
        int bestDIdx = -1;

        // Try all vehicles
        for (size_t v = 0; v < sol.routes.size(); ++v) {
            LocalInsertion insertion = findBestInsertion(sol.routes[v], reqId);

            if (insertion.deltaCost < bestCostIncrease) {
                bestCostIncrease = insertion.deltaCost;
                bestVehicle = (int)v;
                bestPIdx = insertion.pIdx;
                bestDIdx = insertion.dIdx;
            }
        }

        auto& r = sol.routes[bestVehicle];
        sol.insertRequest(reqId, bestVehicle, bestPIdx, bestDIdx, data.N_requests);
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
            LocalInsertion ins = findBestInsertion(sol.routes[v], reqId);
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

            double regret = secondBestCost - bestCost;

            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                winVehicle = bestVForThisReq;
                winMove = bestMoveForThisReq;
            }
        }

        auto& r = sol.routes[winVehicle];
        sol.insertRequest(bestReqId, winVehicle, winMove.pIdx, winMove.dIdx, data.N_requests);
        evaluator.evaluateRoute(r);
        
        // Just update the cache for the affected vehicle
        for (int reqId : sol.unassignedRequests) {
            LocalInsertion ins = findBestInsertion(sol.routes[winVehicle], reqId);
            insertionCache[reqId][winVehicle] = {ins.pIdx, ins.dIdx, ins.deltaCost};
        }
    }
}

void ALNSOperators::repairRegret3(ALNSSolution& sol) {
    int numReqs = data.N_requests;
    int numVehicles = sol.routes.size();

    // Initialize the cache
    insertionCache.assign(numReqs + 1, std::vector<LocalInsertion>(numVehicles));

    for (int reqId : sol.unassignedRequests) {
        for (int v = 0; v < numVehicles; ++v) {
            LocalInsertion ins = findBestInsertion(sol.routes[v], reqId);
            insertionCache[reqId][v] = {ins.pIdx, ins.dIdx, ins.deltaCost};
        }
    }

    while (!sol.unassignedRequests.empty()) {
        int bestReqId = -1;
        double maxRegretValue = -1.0;
        LocalInsertion winMove;
        int winVehicle = -1;

        // Search the request with the highest regret-3 value
        for (int reqId : sol.unassignedRequests) {
            double bestCost = std::numeric_limits<double>::infinity();
            double secondBestCost = std::numeric_limits<double>::infinity();
            double thirdBestCost = std::numeric_limits<double>::infinity();
            int bestVForThisReq = -1;
            LocalInsertion bestMoveForThisReq;

            // Find the 3 best insertions for this request
            for (int v = 0; v < numVehicles; ++v) {
                double cost = insertionCache[reqId][v].deltaCost;
                if (cost < bestCost) {
                    thirdBestCost = secondBestCost;
                    secondBestCost = bestCost;
                    bestCost = cost;
                    bestVForThisReq = v;
                    bestMoveForThisReq = insertionCache[reqId][v];
                } else if (cost < secondBestCost) {
                    thirdBestCost = secondBestCost;
                    secondBestCost = cost;
                } else if (cost < thirdBestCost) {
                    thirdBestCost = cost;
                }
            }

            double regret = 0.0;

            // Calculate regret-3 value based on the number of vehicles available
            if (numVehicles >= 3) {
                regret = (secondBestCost - bestCost) + (thirdBestCost - bestCost);
            } else if (numVehicles == 2) {
                // Fallback to regret-2 if the instance has less than 3 vehicles
                regret = secondBestCost - bestCost;
            } else {
                // If there is only 1 vehicle, regret has no comparative sense
                regret = 0.0;
            }

            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                winVehicle = bestVForThisReq;
                winMove = bestMoveForThisReq;
            }
        }

        // Apply the best move
        auto& r = sol.routes[winVehicle];
        sol.insertRequest(bestReqId, winVehicle, winMove.pIdx, winMove.dIdx, data.N_requests);
        evaluator.evaluateRoute(r);
        
        // Update the cache ONLY for the modified vehicle
        for (int reqId : sol.unassignedRequests) {
            LocalInsertion ins = findBestInsertion(sol.routes[winVehicle], reqId);
            insertionCache[reqId][winVehicle] = {ins.pIdx, ins.dIdx, ins.deltaCost};
        }
    }
}

void ALNSOperators::applyIntraRouteExchanges(ALNSSolution& sol) {
    bool globalImprovement = false;
    int k = params.balasSimonettiK; // Balas-Simonetti parameter

    for (auto& route : sol.routes) {
        // Skip empty routes or routes with only Start -> End
        if (route.sequence.size() <= 2) continue; 

        bool routeImproved = true;
        
        // Keep looping until no further improvements can be found in this route
        while (routeImproved) {
            routeImproved = false;
            double baselineRouteCost = route.totalCost;
            double bestRouteCost = baselineRouteCost;

            int bestNodeToMove = -1;
            int bestInsertPos = -1;

            // Extract a snapshot of the nodes to evaluate (excluding depots)
            std::vector<int> nodesToMove = route.sequence;
            nodesToMove.erase(nodesToMove.begin()); // Remove Start Depot
            nodesToMove.pop_back();                 // Remove End Depot

            // Scan all nodes to find the single best move (Hill Climbing)
            for (int v : nodesToMove) {
                int v_pos = route.getPosition(v);
                int originalPos = v_pos; // Save original position
                if (v_pos == -1) continue; 
                
                int n = data.P.size();
                bool isPickup = data.isPickup(v);
                int partnerId = isPickup ? (v + n) : (v - n);
                int partner_pos = route.getPosition(partnerId);

                // Temporarily remove vertex v from the route
                route.sequence.erase(route.sequence.begin() + v_pos);
                
                // Adjust partner position since we shifted the array down by 1
                if (v_pos < partner_pos)
                    partner_pos--;

                // Determine valid insertion bounds
                int minInsert = std::max(1, originalPos - (k - 1));
                int maxInsert = std::min((int)route.sequence.size() - 1,
                             originalPos + (k - 1));

                if (isPickup) maxInsert = std::min(maxInsert, partner_pos); // Pickup must go BEFORE delivery
                else minInsert = std::max(minInsert, partner_pos + 1); // Delivery must go AFTER pickup

                // Evaluate all valid positions for this specific node
                for (int pos = minInsert; pos <= maxInsert; ++pos) {
                    route.sequence.insert(route.sequence.begin() + pos, v);
                    evaluator.evaluateRoute(route);
                    
                    // Track the absolute best move found across the entire route so far
                    if (route.totalCost < bestRouteCost - 1e-6) {
                        bestRouteCost = route.totalCost;
                        bestNodeToMove = v;
                        bestInsertPos = pos;
                    }
                    
                    // Remove to try the next position
                    route.sequence.erase(route.sequence.begin() + pos);
                }

                // Restore route to its original state so the next node is evaluated against
                // the correct structure
                route.sequence.insert(route.sequence.begin() + v_pos, v);
                evaluator.evaluateRoute(route); 
            }

            // After checking all nodes, commit to the single best neighborhood move
            if (bestNodeToMove != -1) {
                int v_pos = route.getPosition(bestNodeToMove);
                
                route.sequence.erase(route.sequence.begin() + v_pos);
                route.sequence.insert(route.sequence.begin() + bestInsertPos, bestNodeToMove);
                evaluator.evaluateRoute(route); // Update the final route metrics

                routeImproved = true;
                globalImprovement = true;
            }
        }
    }

    // If any route was improved, re-evaluate the full solution to update overall objective and violations
    if (globalImprovement) {
        evaluator.evaluateSolution(sol);
    }
}

double ALNSOperators::calculateRelatedness(int i, int j, const ALNSSolution& sol) {
    int n = data.N_requests;

    // Heuristic weights
    double w_dist = params.shawDistWeight;
    double w_time = params.shawTimeWeight;
    double w_demand = params.shawDemandWeight;


    double dist = data.getTravelTime(i, j) + data.getTravelTime(n + i, n + j);
    double timeDiff = std::abs(sol.getB(i) - sol.getB(j)) +
                      std::abs(sol.getB(n + i) - sol.getB(n + j));    
    double demandDiff = std::abs(data.getDemand(i) - data.getDemand(j));

    // Relatedness value
    return w_dist*dist + w_time*timeDiff + w_demand*demandDiff;
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertion(
        const ALNSRoute& route,
        int reqId
    ) {
    LocalInsertion best;
    
    if (reductionMethod == REDUCTION) {
        return findBestInsertionExact_R(route, reqId);
    }
    else {
        return findBestInsertionExact(route, reqId);
    }
}

ALNSOperators::LocalInsertion ALNSOperators::findBestInsertionExact(const ALNSRoute& route, int reqId) {
    LocalInsertion best;
    int n = route.sequence.size();

    ALNSRoute temp = route;

    for (int i = 1; i < n; ++i) { // Pickup can be inserted between any two nodes
        for (int j = i; j < n; ++j) { // Delivery must come after pickup
            double delta = evaluator.calculateDelta(route, temp, reqId, i, j);
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
            double delta = evaluator.calculateDelta(route, temp, reqId, i, i);
            if (delta < bestDeltaI) {
                bestDeltaI = delta;
                bestI = i;
            }
        }

        // PHASE 2: Maintaining the Pickup fixed (bestI), test all valid positions for the Delivery (j)
        if (bestI != -1) {
            for (int j = bestI; j < n; ++j) {
                double delta = evaluator.calculateDelta(route, temp, reqId, bestI, j);
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
            double delta = evaluator.calculateDelta(route, temp, reqId, j, j);
            if (delta < bestDeltaJ) {
                bestDeltaJ = delta;
                bestJ = j;
            }
        }

        // PHASE 2: Maintaining the Delivery fixed (bestJ), test all valid positions for the Pickup (i)
        if (bestJ != -1) {
            for (int i = 1; i <= bestJ; ++i) {
                double delta = evaluator.calculateDelta(route, temp, reqId, i, bestJ);
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
