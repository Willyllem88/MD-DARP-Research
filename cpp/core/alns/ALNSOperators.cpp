#include "ALNSOperators.h"

#include "../DARPMD_ProblemInstance.h"
#include "ALNSEvaluator.h"
        
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

ALNSOperators::ALNSOperators(const DARPMD_ProblemInstance& instance,
                             const ALNSParams& parameters,
                             ALNSEvaluator& evaluator,
                             std::mt19937& randomEngine)
    : data(instance), params(parameters), evaluator(evaluator), rng(randomEngine) {
}

void ALNSOperators::destroyRandom(ALNSSolution& sol, int q) {
    // Randomly remove q requests
    std::vector<int> requestsInRoutes;
    // Identify where everyone is
    struct Loc { int vIdx; int nodeIdx; int reqId; };
    std::vector<Loc> locations;

    for (size_t v = 0; v < sol.routes.size(); ++v) {
        const auto& seq = sol.routes[v].sequence;
        for (size_t i = 1; i < seq.size() - 1; ++i) {
            // If it's a pickup node
            if (data.isPickup(seq[i])) {
                locations.push_back({(int)v, (int)i, seq[i]});
            }
        }
    }

    std::shuffle(locations.begin(), locations.end(), rng);
    
    int removed = 0;
    for (const auto& loc : locations) {
        if (removed >= q) break;

        // Remove pickup (loc.reqId) and delivery (reqId + N) from route
        auto& route = sol.routes[loc.vIdx];
        
        // Simple removal: iterate backwards to keep indices safe roughly, 
        // or just rebuild vector
        std::vector<int> newSeq;
        int deliveryId = loc.reqId + data.N_requests;
        
        for (int node : route.sequence) {
            if (node != loc.reqId && node != deliveryId) {
                newSeq.push_back(node);
            }
        }
        route.sequence = newSeq;
        
        sol.unassignedRequests.insert(loc.reqId);
        removed++;
    }

    evaluator.evaluateSolution(sol);
}

void ALNSOperators::destroyWorst(ALNSSolution& sol, int q) {
    std::vector<std::pair<double, int>> savingsMap; // <Saving, RequestID>

    // 1. Calculate the cost of each request in the current solution
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
        for (int reqId : requestsInRoute) {
            ALNSRoute tempRoute = route;
            std::vector<int> newSeq;
            int deliveryId = reqId + data.N_requests;

            for (int node : tempRoute.sequence) {
                if (node != reqId && node != deliveryId) newSeq.push_back(node);
            }
            tempRoute.sequence = newSeq;
            evaluator.evaluateRoute(tempRoute);

            double saving = currentCost - tempRoute.totalCost;
            savingsMap.push_back({saving, reqId});
        }
    }

    // 2. Sort by greatest saving (Descending)
    std::sort(savingsMap.rbegin(), savingsMap.rend()); 

    // 3. Remove the top 'q' (with a random factor to avoid pure determinism)
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

double ALNSOperators::calculateRelatedness(int i, int j) {
    // Heurístic weights
    double w_dist = params.shawDistWeight;
    double w_time = params.shawTimeWeight;
    double w_demand = params.shawDemandWeight;

    // Distancia normalizada (aprox) entre orígenes
    double dist = data.getTravelTime(i, j); 
    
    // Diferencia temporal (Start Time Window)
    double mu_i = (data.getTimeWindowStart(i) + data.getTimeWindowEnd(i)) / 2.0;
    double mu_j = (data.getTimeWindowStart(j) + data.getTimeWindowEnd(j)) / 2.0;
    double timeDiff = std::abs(mu_i - mu_j);
    
    // Diferencia de demanda
    double demandDiff = std::abs(data.getDemand(i) - data.getDemand(j));

    // Valor de relación (Shaw Distance)
    return w_dist * dist + w_time * timeDiff + w_demand * demandDiff;
}

void ALNSOperators::destroyShaw(ALNSSolution& sol, int q) {
    // 1. Choose a random seed request that is currently assigned
    const std::vector<int>& assigned = data.P;
    std::uniform_int_distribution<> dis(0, assigned.size() - 1);
    int seedRequest = assigned[dis(rng)];

    std::vector<int> toRemove;
    toRemove.push_back(seedRequest);

    // 2. Sort the rest of the requests by similarity to the seed
    std::vector<std::pair<double, int>> relatedList;
    for (int other : assigned) {
        if (other == seedRequest) continue;
        double R = calculateRelatedness(seedRequest, other);
        relatedList.push_back({R, other});
    }
    std::sort(relatedList.begin(), relatedList.end()); // Lower R is better

    // 3. Select q-1 closest neighbors (with some randomness)
    while ((int)toRemove.size() < q && !relatedList.empty()) {
        double r = std::generate_canonical<double, 10>(rng);
        int idx = std::floor(relatedList.size() * std::pow(r, 6.0)); // Strong bias towards the beginning
        if (idx >= (int)relatedList.size()) idx = (int)relatedList.size() - 1;

        toRemove.push_back(relatedList[idx].second);
        relatedList.erase(relatedList.begin() + idx);
    }

    // 4. Execute removal
    for (int req : toRemove) {
        for (auto& route : sol.routes) {
            // ... Same physical removal code as in destroyRandom/Worst ...
            // (You can refactor the physical removal to an auxiliary function 'removeRequestFromRoute')
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
    // Try to insert unassigned requests into best positions
    // Simple version: iterate unassigned, find best spot, insert, repeat
    
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
            const auto& seq = sol.routes[v].sequence;
            
            // Try all insertion pairs (i, j) where i < j
            // sequence: 0 ... i ... j ... end
            for (size_t i = 1; i < seq.size(); ++i) {
                for (size_t j = i; j < seq.size(); ++j) {
                    
                    double delta = evaluator.calculateDelta(sol.routes[v], reqId, i, j, bestCostIncrease);
                    
                    if (delta < bestCostIncrease) {
                        bestCostIncrease = delta;
                        bestVehicle = v;
                        bestPIdx = i;
                        bestDIdx = j + 1;
                    }
                }
            }
        }

        auto& r = sol.routes[bestVehicle];
        r.sequence.insert(r.sequence.begin() + bestPIdx, reqId);
        r.sequence.insert(r.sequence.begin() + bestDIdx, deliveryId);

        evaluator.evaluateRoute(r);
    }
}

void ALNSOperators::repairRegret2(ALNSSolution& sol) {
    while (!sol.unassignedRequests.empty()) {
        
        int bestReqId = -1;
        double maxRegretValue = -1.0;
        
        // Structure to store the winning move for the best request
        int winVehicle = -1; 
        int winPIdx = -1; 
        int winDIdx = -1;

        // Iterate over all unassigned requests
        std::vector<int> pending(sol.unassignedRequests.begin(), sol.unassignedRequests.end());
        
        for (int reqId : pending) {            
            // Store the insertion costs of this request in each route
            std::vector<double> insertionCosts; 
            
            // Temporary structures to store the exact position in each route
            struct Move { int v; int pIdx; int dIdx; double cost; };
            std::vector<Move> moves;

            // Evaluate insertion in EACH vehicle
            for (size_t v = 0; v < sol.routes.size(); ++v) {
                double bestCostForVehicle = std::numeric_limits<double>::max();
                int bestP = -1, bestD = -1;
                
                // LLogic for position search (same as in Greedy)
                const auto& seq = sol.routes[v].sequence;

                for (size_t i = 1; i < seq.size(); ++i) {
                    for (size_t j = i; j < seq.size(); ++j) {
                        double delta = evaluator.calculateDelta(sol.routes[v], reqId, i, j, bestCostForVehicle);
                        if (delta < bestCostForVehicle) {
                            bestCostForVehicle = delta;
                            bestP = i;
                            bestD = j + 1;
                        }
                    }
                }
                
                // There will always be at least one possible insertion
                moves.push_back({(int)v, bestP, bestD, bestCostForVehicle});
            }
            
            // Sort moves by cost (ascending)
            std::sort(moves.begin(), moves.end(), [](const Move& a, const Move& b){
                return a.cost < b.cost;
            });

            double regret = 0;
            if (moves.size() == 1) {
                // If it only fits in one place, the regret is infinite (or very high)
                // because if we don't put it there, we lose it.
                regret = 100000.0; // High arbitrary value to prioritize this request
            } else {
                // Regret-2: the difference between the best and second-best insertion cost
                regret = moves[1].cost - moves[0].cost;
            }

            // Is this the "worst" case we have seen?
            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                winVehicle = moves[0].v;
                winPIdx = moves[0].pIdx;
                winDIdx = moves[0].dIdx;
            }
        }

        // Apply the winning move for the request with the highest regret
        auto& r = sol.routes[winVehicle];
        r.sequence.insert(r.sequence.begin() + winPIdx, bestReqId);
        int deliveryId = bestReqId + data.N_requests;
        r.sequence.insert(r.sequence.begin() + winDIdx, deliveryId);
        evaluator.evaluateRoute(r);
        
        sol.unassignedRequests.erase(bestReqId);
    }
}