#include "ALNSSolver.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

#include "CPLEXSoftSolver.h"

ALNSSolver::ALNSSolver(DARPMD_ProblemInstance& instance,
                       std::optional<double> timeLimit,
                       int seed,
                       bool verbose)
    : Solver(verbose), data(instance), timeLimit(timeLimit) {

    rng = std::mt19937(seed);

    params = std::make_unique<ALNSParams>();
    evaluator = std::make_unique<ALNSEvaluator>(data, *params);
    spSolver = std::make_unique<SetPartitioningSolver>(data, *params, *evaluator, logger);
    operators = std::make_unique<ALNSOperators>(data, *params, *evaluator, rng);

    bestObjective = std::numeric_limits<double>::infinity();
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
    ALNSSolution spSol = spSolver->solve(routePool);

    if (spSol.routes.empty() && spSol.unassignedRequests.empty() && spSol.objectiveValue == 0) {
        return;
    }

    // Handle the new solution from CPLEX
    if (spSol.objectiveValue < bestObjective) {
        logger.log("  [Matheuristic] CPLEX found new best solution! Objective: " + std::to_string(spSol.objectiveValue) + " (Improvement)");
        
        bestSolution = spSol;
        bestObjective = spSol.objectiveValue;
    }
    else {
        logger.log("  [Matheuristic] CPLEX found solution with objective: " + std::to_string(spSol.objectiveValue) + " (No improvement)");
    }
}

void ALNSSolver::solveScheduleLater(ALNSSolution& sol) {
    logger.log("[Schedule Later] Starting Schedule Later phase with CPLEX. Current solution objective: " + std::to_string(sol.objectiveValue));
    CPLEXSoftSolver softSolver(data, std::nullopt, verbose);

    softSolver.fixAllRoutingVariablesToZero();

    for (const auto& route : sol.routes) {
        for (size_t i = 0; i < route.sequence.size() - 1; ++i) {
            int from = route.sequence[i];
            int to = route.sequence[i + 1];
            int k = route.vehicleId;

            softSolver.fixRoutingVariable(from, to, k, 1.0);
        }
    }

    softSolver.solve();
    DARPMD_ResultInstance result = softSolver.getResult();
 
    if (result.objectiveValue < bestObjective) {
        logger.log("[Schedule Later] CPLEX improved the solution! Objective: " + std::to_string(result.objectiveValue) + " (Improvement)");
        bestObjective = result.objectiveValue;
    } else {
        logger.log("[Schedule Later] CPLEX found solution with objective: " + std::to_string(result.objectiveValue) + " (No improvement)");
    }
}

ALNSSolution ALNSSolver::createInitialSolution() {
    ALNSSolution sol;
    
    // Initialize empty routes
    for (int k : data.K) {
        ALNSRoute r;
        r.vehicleId = k;
        r.sequence.push_back(data.getVehicleStartNode(k));
        r.sequence.push_back(data.getVehicleEndNode(k));
        evaluator->evaluateRoute(r); // Zero cost initially
        sol.routes.push_back(r);
    }

    // All requests start as unassigned
    for (int i : data.P) {
        sol.unassignedRequests.insert(i);
    }
    
    // Apply greedy repair to insert as many as possible
    operators->repairGreedy(sol);
    evaluator->evaluateSolution(sol);
    
    return sol;
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
            stats.weights[i] = (1.0 - params->reactionFactor) * stats.weights[i] 
                             + params->reactionFactor * avgScore;
        }
        // Resetear scores para el siguiente segmento
        stats.scores[i] = 0.0;
        stats.timesUsed[i] = 0;
    }
}

bool ALNSSolver::stoppingCriteria(int iter, double elapsedSeconds) {
    // 1. Time limit
    if (timeLimit.has_value() && elapsedSeconds >= timeLimit.value()) {
        logger.log("--- Stopping: Time limit reached (" + std::to_string(elapsedSeconds) + "s) ---");
        return true;
    }

    // 2. Iteration limit
    if (iter >= params->maxIterations) {
        logger.log("--- Stopping: Max iterations reached (" + std::to_string(iter) + ") ---");
        return true;
    }

    // 3. TODO: No improvement for X iterations

    return false;
}

bool ALNSSolver::acceptanceCriteria(double neighborObj, double currentObj, double temperature, double& score) {
    double delta = neighborObj - currentObj;

    // Caso 1: Mejora la solución actual
    if (delta < 0) {
        // Si además mejora la mejor global
        if (neighborObj < bestObjective) {
            score = params->sigma1; // Nueva mejor global
        } else {
            score = params->sigma2; // Mejora actual pero no global
        }
        return true;
    } 
    
    // Caso 2: Solución peor (Criterio de Simulated Annealing)
    double prob = std::exp(-delta / temperature);
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    if (dis(rng) < prob) {
        score = params->sigma3; // Aceptada siendo peor
        return true;
    }

    // Caso 3: Rechazada
    score = 0.0; 
    return false;
}

void ALNSSolver::applyDestroy(ALNSSolution& sol, int destroyOpIdx) {
    double q = std::clamp((int)(params->destroyFraction * data.P.size()), 1, (int)data.P.size() - 1);

    switch ((DestroyMethod)destroyOpIdx) {
        case DestroyMethod::RANDOM:
            operators->destroyRandom(sol, q);
            break;
        case DestroyMethod::WORST:
            operators->destroyWorst(sol, q);
            break;
        case DestroyMethod::SHAW:
            operators->destroyShaw(sol, q);
            break;
        case DestroyMethod::COUNT:
            break; // Should not happen, just a placeholder for counting
    }
}

void ALNSSolver::applyRepair(ALNSSolution& sol, int repairOpIdx) {
    switch ((RepairMethod)repairOpIdx) {
        case RepairMethod::GREEDY:
            operators->repairGreedy(sol);
            break;
        case RepairMethod::REGRET2:
            operators->repairRegret2(sol);
            break;
        case RepairMethod::COUNT:
            break;
    }
}

// ------------------------------------------------------------------
// Main Solve Loop
// ------------------------------------------------------------------
void ALNSSolver::solve() {
    logger.log("Starting ALNS search");

    // 1. Initial Solution
    auto start = std::chrono::high_resolution_clock::now();
    ALNSSolution currentSol = createInitialSolution();
    bestSolution = currentSol;
    bestObjective = currentSol.objectiveValue;

    destroyStats.init((int)DestroyMethod::COUNT);
    repairStats.init((int)RepairMethod::COUNT);

    double z0 = currentSol.objectiveValue;
    double initialTemperature = (params->w * z0) / std::log(2.0); // Accept 10% worst solution at the beggining with probability of 50%
    double temperature = initialTemperature;

    logger.log("Initial solution created. Objective: " + std::to_string(bestObjective) 
        + " (Violations: " + (evaluator->solutionHasViolations(currentSol) ? "Yes" : "No") + ")");

    // 2. Main Loop
    for (int iter = 0; ; ++iter) {
        
        // Check time limit
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (stoppingCriteria(iter, elapsed.count())) break;

        // Adaptive Operator Selection
        int destroyOpIdx = selectOperator(destroyStats.weights);
        int repairOpIdx = selectOperator(repairStats.weights);
        ALNSSolution neighbor = currentSol;

        // --- Destroy ---
        applyDestroy(neighbor, destroyOpIdx);
        // --- Repair ---
        applyRepair(neighbor, repairOpIdx);

        evaluator->evaluateSolution(neighbor);
        for (const auto& route : neighbor.routes) {
            if (!route.sequence.empty()) {
                addRouteToPool(route);
            }
        }

        // --- Acceptance Criterion ---
        double iterScore = 0.0;
        bool accept = acceptanceCriteria(neighbor.objectiveValue, currentSol.objectiveValue, temperature, iterScore);

        if (accept) {
            currentSol = neighbor;

            if (neighbor.objectiveValue < bestObjective) {
                bestSolution = neighbor;
                bestObjective = neighbor.objectiveValue;
                logger.log("* Iter " + std::to_string(iter) + ": New Best = " + std::to_string(bestObjective) 
                    + " (Violations: " + (evaluator->solutionHasViolations(neighbor) ? "Yes" : "No") + ")");
                //solutionDetails(bestSolution);
            }

            checkPickupAfterDelivery(currentSol, data);
        }
        
        // -- Update Operator Stats ---
        destroyStats.scores[destroyOpIdx] += iterScore;
        destroyStats.timesUsed[destroyOpIdx] += 1;
        repairStats.scores[repairOpIdx] += iterScore;
        repairStats.timesUsed[repairOpIdx] += 1;

        if (iter > 0 && iter % 100 == 0) {
            updateWeights(destroyStats);
            updateWeights(repairStats);
        }

        // --- Cooling ---
        temperature *= params->coolingRate;

        // --- Matheuristic Integration ---
        if (iter > 0 && iter % params->setPartitioningInterval == 0) {
            logger.log("Iter " + std::to_string(iter) + " [Matheuristic] Running Set Partitioning on Pool...");
            solveSetPartitioning();
        }
    }

    // Final clean run of SP
    logger.log("Final Set Partitioning to polish solution...");
    solveSetPartitioning();

    // Solve the schedule later
    solveScheduleLater(bestSolution);
    // TODO: this must be the new best solution

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = end - start;
    this->solveTime = totalElapsed.count();

    std::cout << std::endl << std::string(50, '=') << std::endl 
              << "ALNS Finished. Best Objective: " << bestObjective << std::endl;
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
        std::cout << "  Vehicle " << r.vehicleId << " Violations: TimeWindows: " << r.timeWindowViolation
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
        std::map<int, size_t> pickupPositions; // request ID -> position
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
                    logger.log("WARNING: Delivery " + std::to_string(node) + " appears before Pickup " 
                        + std::to_string(pickupId) + " in Vehicle " + std::to_string(route.vehicleId));
                }
            }
        }
    }
}

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
            step.arrivalTime = (nodeId < (int)r.arrivalTimes.size()) ? r.arrivalTimes[nodeId] : 0.0;
            step.loadAfter   = (nodeId < (int)r.loads.size())        ? r.loads[nodeId]        : 0.0;

            vRoute.steps.push_back(step);
        }
        result.addRoute(r.vehicleId, vRoute);
    }

    return result;
}
