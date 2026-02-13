#include "ALNSSolver.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

ALNSSolver::ALNSSolver(const DARPMD_ProblemInstance& instance,
                       std::optional<double> timeLimit)
    : data(instance),
      timeLimit(timeLimit),
      evaluator(*this, data, params),
      spSolver(data, params, evaluator),
      bestObjective(std::numeric_limits<double>::infinity())
{
    rng = std::mt19937(123);
}

ALNSSolver::~ALNSSolver() {
}

// Set Partitioning using CPLEX
void ALNSSolver::addRouteToPool(const ALNSRoute& route) {
    if (route.sequence.empty()) return;
    auto &seen = seenRoutes[route.vehicleId];

    // Try to insert the route sequence into the seen set for this vehicle,
    // returns true if it was not already present
    auto result = seen.insert(route.sequence);

    if (result.second) // If this sequence was not seen before
        routePool[route.vehicleId].push_back(route);

}

void ALNSSolver::solveSetPartitioning() {
    ALNSSolution spSol = spSolver.solve(routePool);

    if (spSol.routes.empty() && spSol.unassignedRequests.empty() && spSol.objectiveValue == 0) {
        return;
    }

    // Handle the new solution from CPLEX
    if (spSol.objectiveValue < bestObjective) {
        std::cout << "  [Matheuristic] CPLEX found new best: " << spSol.objectiveValue 
                  << "  (Improvement)" << std::endl;
        
        bestSolution = spSol;
        bestObjective = spSol.objectiveValue;
    }
    else {
        std::cout << "  [Matheuristic] CPLEX found solution with objective: " << spSol.objectiveValue 
                  << "  (No improvement)" << std::endl;
    }
}

// ------------------------------------------------------------------
// High-Level ALNS Methods
// ------------------------------------------------------------------

ALNSSolution ALNSSolver::createInitialSolution() {
    ALNSSolution sol;
    
    // Initialize empty routes
    for (int k : data.K) {
        ALNSRoute r;
        r.vehicleId = k;
        r.sequence.push_back(data.StartNode.at(k));
        r.sequence.push_back(data.EndNode.at(k));
        evaluator.evaluateRoute(r); // Zero cost initially
        sol.routes.push_back(r);
    }

    // All requests start as unassigned
    for (int i : data.P) {
        sol.unassignedRequests.insert(i);
    }
    
    // Apply greedy repair to insert as many as possible
    repairGreedy(sol);
    evaluator.evaluateSolution(sol);
    
    return sol;
}

// ------------------------------------------------------------------------------------
//                                 DESTROY OPERATORS
// ------------------------------------------------------------------------------------

void ALNSSolver::destroyRandom(ALNSSolution& sol, int q) {
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
}

// TODO: refactor, once it removes one request, already calculated saving are not valid.
//  We could recalculate savings after each removal, but that would be costly. A middle 
//  ground is to recalculate savings every X removals or to accept some noise in the selection.
void ALNSSolver::destroyWorst(ALNSSolution& sol, int q) {
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
        
        if (idx >= savingsMap.size()) idx = savingsMap.size() - 1;

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
}

void ALNSSolver::destroyShaw(ALNSSolution& sol, int q) {
    // 1. Choose a random seed request that is currently assigned
    std::vector<int> assigned;
    for (const auto& r : sol.routes) {
        for (int node : r.sequence) {
            if (data.isPickup(node)) 
                assigned.push_back(node);
        }
    }
    
    if (assigned.empty()) return;
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
    while (toRemove.size() < q && !relatedList.empty()) {
        double r = std::generate_canonical<double, 10>(rng);
        int idx = std::floor(relatedList.size() * std::pow(r, 6.0)); // Strong bias towards the beginning
        if (idx >= relatedList.size()) idx = relatedList.size() - 1;

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
}

// -----------------------------------------------------------------------------
//                                 REPAIR OPERATORS
// -----------------------------------------------------------------------------

void ALNSSolver::repairGreedy(ALNSSolution& sol) {
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
                    
                    // Construct temp route
                    ALNSRoute temp = sol.routes[v];
                    temp.sequence.insert(temp.sequence.begin() + i, reqId);
                    temp.sequence.insert(temp.sequence.begin() + j + 1, deliveryId);
                    
                    evaluator.evaluateRoute(temp);
                    
                    double increase = temp.totalCost - sol.routes[v].totalCost;
                    
                    if (increase < bestCostIncrease) {
                        bestCostIncrease = increase;
                        bestVehicle = v;
                        bestPIdx = i;
                        bestDIdx = j + 1;
                    }
                }
            }
        }

        // Apply best move if reasonable
        if (bestVehicle != -1 && bestCostIncrease < params.unassignedPenalty) {
            auto& r = sol.routes[bestVehicle];
            r.sequence.insert(r.sequence.begin() + bestPIdx, reqId);
            r.sequence.insert(r.sequence.begin() + bestDIdx, deliveryId);
            // Don't need to eval full solution yet, loop handles local delta
            // But we should update the route cost locally for next comparison
            evaluator.evaluateRoute(r);
        } else {
            sol.unassignedRequests.insert(reqId);
        }
    }
}

void ALNSSolver::repairRegret2(ALNSSolution& sol) {
    while (!sol.unassignedRequests.empty()) {
        
        int bestReqId = -1;
        double maxRegretValue = -1.0;
        
        // Estructura para guardar el mejor movimiento de la petición ganadora
        int winVehicle = -1; 
        int winPIdx = -1; 
        int winDIdx = -1;

        // Iterar sobre todas las peticiones pendientes
        std::vector<int> pending(sol.unassignedRequests.begin(), sol.unassignedRequests.end());
        
        for (int reqId : pending) {
            int deliveryId = reqId + data.N_requests;
            
            // Guardar los costes de inserción de esta petición en cada ruta
            std::vector<double> insertionCosts; 
            
            // Estructuras temporales para guardar la posición exacta en cada ruta
            struct Move { int v; int pIdx; int dIdx; double cost; };
            std::vector<Move> moves;

            // Evaluar inserción en CADA vehículo
            for (size_t v = 0; v < sol.routes.size(); ++v) {
                double bestCostForVehicle = std::numeric_limits<double>::max();
                int bestP = -1, bestD = -1;
                
                // Lógica de búsqueda de posición (igual que en Greedy)
                const auto& seq = sol.routes[v].sequence;
                double currentRouteCost = sol.routes[v].totalCost;

                for (size_t i = 1; i < seq.size(); ++i) {
                    for (size_t j = i; j < seq.size(); ++j) {
                        // Delta Evaluation rápida o clonación completa (Usamos clonación por simplicidad actual)
                        ALNSRoute temp = sol.routes[v];
                        temp.sequence.insert(temp.sequence.begin() + i, reqId);
                        temp.sequence.insert(temp.sequence.begin() + j + 1, deliveryId);
                        evaluator.evaluateRoute(temp);
                        
                        if (!temp.isFeasible) continue; // Si viola Hard Constraints, ignorar
                        
                        double increase = temp.totalCost - currentRouteCost;
                        if (increase < bestCostForVehicle) {
                            bestCostForVehicle = increase;
                            bestP = i;
                            bestD = j + 1;
                        }
                    }
                }
                
                // Guardamos el mejor coste encontrado para este vehículo
                if (bestP != -1) {
                    moves.push_back({(int)v, bestP, bestD, bestCostForVehicle});
                }
            }

            // CALCULAR REGRET
            // Si no cabe en ningún sitio
            if (moves.empty()) continue; 
            
            // Ordenar movimientos por coste (ascendente)
            std::sort(moves.begin(), moves.end(), [](const Move& a, const Move& b){
                return a.cost < b.cost;
            });

            double regret = 0;
            if (moves.size() == 1) {
                // Si solo cabe en un sitio, el arrepentimiento es infinito (o muy alto)
                // porque si no lo metemos ahí, lo perdemos.
                regret = 100000.0; // Valor alto arbitrario
            } else {
                // Regret-2: Diferencia entre mejor y segundo mejor
                regret = moves[1].cost - moves[0].cost;
            }

            // ¿Es este el "peor" caso que hemos visto?
            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                winVehicle = moves[0].v;
                winPIdx = moves[0].pIdx;
                winDIdx = moves[0].dIdx;
            }
        }

        // Si no encontramos candidato factible para ninguna petición, paramos
        if (bestReqId == -1) break;

        // APLICAR EL MOVIMIENTO
        auto& r = sol.routes[winVehicle];
        r.sequence.insert(r.sequence.begin() + winPIdx, bestReqId);
        int deliveryId = bestReqId + data.N_requests;
        r.sequence.insert(r.sequence.begin() + winDIdx, deliveryId);
        evaluator.evaluateRoute(r);
        
        sol.unassignedRequests.erase(bestReqId);
    }
}

int ALNSSolver::selectOperator(const std::vector<double>& weights) {
    double totalWeight = 0.0;
    for (double w : weights) totalWeight += w;

    std::uniform_real_distribution<> dis(0.0, totalWeight);
    double r = dis(rng);
    
    double cumulative = 0.0;
    for (size_t i = 0; i < weights.size(); ++i) {
        cumulative += weights[i];
        if (r <= cumulative) {
            return (int)i;
        }
    }
    return (int)weights.size() - 1; // Fallback
}

void ALNSSolver::updateWeights(OperatorStats& stats) {
    for (size_t i = 0; i < stats.weights.size(); ++i) {
        if (stats.timesUsed[i] > 0) {
            double avgScore = stats.scores[i] / stats.timesUsed[i];
            // Fórmula: w_new = (1-p) * w_old + p * score
            stats.weights[i] = (1.0 - params.reactionFactor) * stats.weights[i] 
                             + params.reactionFactor * avgScore;
        }
        // Resetear scores para el siguiente segmento
        stats.scores[i] = 0.0;
        stats.timesUsed[i] = 0;
    }
}

double ALNSSolver::calculateRelatedness(int i, int j) {
    // Pesos heurísticos
    double w_dist = 9.0;
    double w_time = 3.0;
    double w_demand = 1.0;

    // Distancia normalizada (aprox) entre orígenes
    double dist = data.getTravelTime(i, j); 
    
    // Diferencia temporal (Start Time Window)
    double timeDiff = std::abs(data.getTimeWindowStart(i) - data.getTimeWindowStart(j));
    
    // Diferencia de demanda
    double demandDiff = std::abs(data.getDemand(i) - data.getDemand(j));

    // Valor de relación (Shaw Distance)
    return w_dist * dist + w_time * timeDiff + w_demand * demandDiff;
}

// TODO: refactor, a fuction that returns true if the stopping criterion is 
// met (time, iterations, etc)

// ------------------------------------------------------------------
// Main Solve Loop
// ------------------------------------------------------------------
void ALNSSolver::solve() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Starting ALNS search..." << std::endl;

    // 1. Initial Solution
    ALNSSolution currentSol = createInitialSolution();
    bestSolution = currentSol;
    bestObjective = currentSol.objectiveValue;

    destroyStats.init((int)DestroyMethod::COUNT);
    repairStats.init((int)RepairMethod::COUNT);

    double temperature = params.initialTemperature;

    std::cout << "Initial Objective: " << bestObjective << std::endl;

    // 2. Main Loop
    for (int iter = 0; iter < params.maxIterations; ++iter) {
        
        // Check time limit
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (timeLimit.has_value() && elapsed.count() > timeLimit.value()) {
            std::cout << "Time limit reached." << std::endl;
            break;
        }

        // Adaptive Operator Selection
        int destroyOpIdx = selectOperator(destroyStats.weights);
        int repairOpIdx = selectOperator(repairStats.weights);

        // Copy current
        ALNSSolution neighbor = currentSol;

        // --- Destroy ---
        // Destroy q requests, where q is a fraction of total requests (at least 1, at most n-1)
        int q = std::clamp((int)(params.destroyFraction * data.P.size()), 1, (int)data.P.size() - 1);
        switch ((DestroyMethod)destroyOpIdx) {
            case DestroyMethod::RANDOM:
                destroyRandom(neighbor, q);
                break;
            case DestroyMethod::WORST:
                destroyWorst(neighbor, q);
                break;
            case DestroyMethod::SHAW:
                destroyShaw(neighbor, q);
                break;
        }

        // --- Repair ---
        switch ((RepairMethod)repairOpIdx) {
            case RepairMethod::GREEDY:
                repairGreedy(neighbor);
                break;
            case RepairMethod::REGRET2:
                repairRegret2(neighbor);
                break;
        }

        evaluator.evaluateSolution(neighbor);

        // --- Acceptance Criterion ---
        double score = 0.0;
        double delta = neighbor.objectiveValue - currentSol.objectiveValue;
        bool accept = false;
        
        if (delta < 0) {
            accept = true;
            if (neighbor.objectiveValue < bestObjective) {
                bestSolution = neighbor;
                bestObjective = neighbor.objectiveValue;
                score = params.sigma1;
                bool violations = evaluator.solutionHasViolations(neighbor);
                std::cout << "Iter " << iter << ": New Best = " << bestObjective 
                          << " (Violations: " << (violations ? "Yes" : "No") << ")" << std::endl;
                //solutionDetails(bestSolution);
            } else {
                score = params.sigma2;
            }
        } else {
            // Simulated Annealing acceptance
            double prob = std::exp(-delta / temperature);
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(rng) < prob) {
                accept = true;
                score = params.sigma3;
            }
        }

        if (accept) {
            currentSol = neighbor;
            // Check for delivery before pickup violations
            checkPickupAfterDelivery(currentSol, data);
        }

        // -- Update Operator Stats ---
        destroyStats.scores[destroyOpIdx] += score;
        destroyStats.timesUsed[destroyOpIdx] += 1;
        repairStats.scores[repairOpIdx] += score;
        repairStats.timesUsed[repairOpIdx] += 1;

        if (iter > 0 && iter % 100 == 0) {
            updateWeights(destroyStats);
            updateWeights(repairStats);
        }

        // --- Cooling ---
        temperature *= params.coolingRate;

        // --- Matheuristic Integration ---
        if (iter > 0 && iter % params.setPartitioningInterval == 0) {
            std::cout << "Iter " << iter << " [Matheuristic] Running Set Partitioning on Pool..." << std::endl;
            solveSetPartitioning();
            // Optional: Re-inject the CPLEX solution as the current solution for ALNS
            // currentSol = bestSolution; 
        }
    }

    // Final clean run of SP
    std::cout << "Final Set Partitioning to polish solution..." << std::endl;
    solveSetPartitioning();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = end - start;
    this->solveTime = totalElapsed.count();

    //Print the violations in the best solution
    std::cout << std::endl << "ALNS Finished. Best Objective: " << bestObjective << std::endl;
    // Print the objective value of the arcs used (distance cost only, without penalties)
    double totalDistanceCost = 0.0;
    for (const auto& r : bestSolution.routes) {
        totalDistanceCost += r.distanceCost;
    }
    std::cout << "Objective (without penalties): " << totalDistanceCost << std::endl;
    std::cout << "Total Solve Time: " << this->solveTime << " seconds." << std::endl;

    // Print operator stats
    std::cout << std::endl << "Operator Usage Stats:" << std::endl;
    std::cout << "Destroy Operator Stats:" << std::endl;
    for (size_t i = 0; i < destroyStats.weights.size(); ++i) {
        double avgScore = (destroyStats.timesUsed[i] > 0) ? destroyStats.scores[i] / destroyStats.timesUsed[i] : 0.0;
        std::cout << "  Destroy " << i << ": Weight=" << destroyStats.weights[i] 
                  << ", Times Used=" << destroyStats.timesUsed[i] 
                  << ", Avg Score=" << avgScore << std::endl;
    }

    std::cout << "Repair Operator Stats:" << std::endl;
    for (size_t i = 0; i < repairStats.weights.size(); ++i) {
        double avgScore = (repairStats.timesUsed[i] > 0) ? repairStats.scores[i] / repairStats.timesUsed[i] : 0.0;
        std::cout << "  Repair " << i << ": Weight=" << repairStats.weights[i] 
                  << ", Times Used=" << repairStats.timesUsed[i] 
                  << ", Avg Score=" << avgScore << std::endl;
    }

    //Print violation
    std::cout << std::endl << "Violations in Best Solution:" << std::endl;
    for (const auto& r : bestSolution.routes) {
        std::cout << " Vehicle " << r.vehicleId << " Violations: TimeWindows: " << r.timeWindowViolation
                  << ", MaxRouteTime: " << r.vehicleMaxRouteTimeViolation << ", Load: " << r.loadViolation 
                  << ", RideTime: " << r.rideTimeViolation << std::endl;
    }
    // Print unassigned requests
    std::cout << "Unassigned Requests: ";
    if (bestSolution.unassignedRequests.empty()) std::cout << "NONE";
    for (int req : bestSolution.unassignedRequests) {
        std::cout << req << " ";
    }
    std::cout << std::endl;
}

void ALNSSolver::checkPickupAfterDelivery(const ALNSSolution& sol, const DARPMD_ProblemInstance& data) const {
    for (const auto& route : sol.routes) {
        std::map<int, int> pickupPositions; // request ID -> position
        for (size_t pos = 0; pos < route.sequence.size(); ++pos) {
            int node = route.sequence[pos];
            if (data.isPickup(node)) {
                pickupPositions[node] = pos;
            }
        }
        
        for (size_t pos = 0; pos < route.sequence.size(); ++pos) {
            int node = route.sequence[pos];
            if (data.isDelivery(node)) {
                int pickupId = node - data.N_requests;
                if (pickupPositions.count(pickupId) && pickupPositions[pickupId] > pos) {
                    std::cout << "WARNING: Delivery " << node << " appears before Pickup " 
                                << pickupId << " in Vehicle " << route.vehicleId << std::endl;
                }
            }
        }
    }
}

void ALNSSolver::printSolutionDetails(const ALNSSolution& sol) const {
    std::cout << "  Routes:" << std::endl;
    for (const auto& r : sol.routes) {
        std::cout << "    Vehicle " << r.vehicleId << ": ";
        for (int node : r.sequence) {
            std::cout << node << " ";
        }
        std::cout << "| Cost: " << r.totalCost 
                  << " | TW Violation: " << r.timeWindowViolation 
                  << " | Load Violation: " << r.loadViolation 
                  << " | Ride Time Violation: " << r.rideTimeViolation
                  << std::endl;
    }
    std::cout << "  Unassigned Requests: ";
    for (int req : sol.unassignedRequests) {
        std::cout << req << " ";
    }
    std::cout << std::endl;
}


// ------------------------------------------------------------------
// Result Formatting
// ------------------------------------------------------------------
DARPMD_ResultInstance ALNSSolver::getResult() const {
    DARPMD_ResultInstance result(data);
    result.solveTime = this->solveTime;
    result.objectiveValue = this->bestObjective;
    
    // If we have violations, mark as Feasible only (or Infeasible)
    // But since this is heuristic result, we return what we have.
    result.solverStatus = (bestSolution.unassignedRequests.empty()) ? "Feasible" : "Partial/Infeasible";

    for (const auto& r : bestSolution.routes) {
        VehicleRoute vRoute;
        vRoute.vehicleId = r.vehicleId;

        for (int nodeId : r.sequence) {
            RouteStep step;
            step.nodeId = nodeId;
            
            // Types
            if (data.isVehicleStart(nodeId)) step.type = "DepotStart";
            else if (data.isVehicleEnd(nodeId)) step.type = "DepotEnd";
            else if (data.isPickup(nodeId)) step.type = "Pickup";
            else step.type = "Delivery";

            // Timing info from evaluation
            if (r.arrivalTimes.count(nodeId)) step.arrivalTime = r.arrivalTimes.at(nodeId);
            else step.arrivalTime = 0.0;

            if (r.loads.count(nodeId)) step.loadAfter = r.loads.at(nodeId);
            else step.loadAfter = 0.0;

            vRoute.steps.push_back(step);
        }
        result.addRoute(r.vehicleId, vRoute);
    }

    return result;
}

//TODO: Vehicle time violations