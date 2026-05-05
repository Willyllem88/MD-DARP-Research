#include "ALNSSolver.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

#include "CPLEXSoftSolver.h"

ALNSSolver::ALNSSolver(DARPMD_ProblemInstance& instance,
                       std::optional<double> timeLimit,
                       HybridMethod hybridMethod,
                       int seed,
                       bool verbose,
                       const ALNSParams& alnsParams,
                       bool enableGICE,
                       bool enableNR
                       )
    : Solver(verbose), data(instance), timeLimit(timeLimit), hybridMethod(hybridMethod) {

    rng = std::mt19937(seed);

    params = std::make_unique<ALNSParams>(alnsParams);
    evaluator = std::make_unique<ALNSEvaluator>(data, *params);
    operators = std::make_unique<ALNSOperators>(data, *params, *evaluator, rng, enableGICE, enableNR);

    if (hybridMethod == HybridMethod::SET_PARTITIONING) {
        setSolver = std::make_unique<SetPartitioningSolver>(data, *params, *evaluator, logger);
    } else if (hybridMethod == HybridMethod::SET_COVERING) {
        setSolver = std::make_unique<SetCoveringSolver>(data, *params, *evaluator, logger);
    }

    bestObjective = std::numeric_limits<double>::infinity();
}

ALNSSolver::~ALNSSolver() {
}

// Set Partitioning using CPLEX
void ALNSSolver::addRouteToPool(const ALNSRoute& route) {
    if (setSolver) {
        setSolver->getRoutePool().addRoute(route, bestObjective);
    }
}

void ALNSSolver::solveMatheuristic() {
    if (hybridMethod == HybridMethod::NONE) return;

    ALNSSolution matSol;
    setSolver->getRoutePool().prune(bestObjective);
    bool solved = setSolver->solve(matSol);

    if (!solved) {
        logger.log("Iter " + std::to_string(iteration) + "  [Matheuristic] Failed to solve with CPLEX.");
        return;
    }

    updateBestSolutions(matSol, "  [Matheuristic] ");
}

DARPMD_ResultInstance ALNSSolver::solveScheduleLater(ALNSSolution& sol) {
    logger.log("[Schedule Later] Starting... Current solution objective: " + std::to_string(sol.objectiveValue));
    CPLEXSoftSolver softSolver(data, std::nullopt, false);

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

    if (result.objectiveValue < bestObjective - 1e-6) {
        logger.log("[Schedule Later] CPLEX improved the solution! Objective: " + std::to_string(result.objectiveValue) + " (Improvement)");
        bestObjective = result.objectiveValue;
    } else {
        logger.log("[Schedule Later] CPLEX found solution with objective: " + std::to_string(result.objectiveValue) + " (No improvement)");
    }
    
    // NRVO: the compiler optimize this copy
    return result;
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

    return false;
}

bool ALNSSolver::acceptanceCriteria(double neighborObj, double currentObj, double temperature, bool isNew, double& score) {
    double delta = neighborObj - currentObj;

    // Case 1: Improves the current solution
    if (delta < 0) {
        // If also improves the global best
        if (neighborObj < bestObjective) score = params->sigma1; 
        // Improves the current but not the global best, but is a new solution (not visited before)
        else if (isNew) score = params->sigma2;
        // Improves the current but is not a new solution (already visited)
        else score = 0.0;
        return true;
    } 
    
    // Case 2: Worse solution (Simulated Annealing acceptance)
    double prob = std::exp(-delta / temperature);
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    if (dis(rng) < prob) {
        // Accepted although worse
        if (isNew) score = params->sigma3;
        // Accepted but already visited, so no score
        else score = 0.0;
        return true;
    }

    // Case 3: Not accepted
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

void ALNSSolver::initializeStatsAndTemperature(const ALNSSolution& initialSolution) {
    destroyStats.init((int)DestroyMethod::COUNT);
    repairStats.init((int)RepairMethod::COUNT);

    // Accept 10% worst solution at the beggining with probability of 50%
    double z0 = initialSolution.objectiveValue;
    double initialTemperature = (params->w * z0) / std::log(2.0);
    currentTemperature = initialTemperature;

    logger.log("Initial solution created. Objective: " + std::to_string(bestObjective) 
        + " (Violations: " + (initialSolution.hasViolations ? "Yes" : "No") + ")");
}

void ALNSSolver::initializeRoutePool() {
    ALNSSolution emptySol;
    for (int k : data.K) {
        ALNSRoute r;
        r.vehicleId = k;
        r.sequence.push_back(data.getVehicleStartNode(k));
        r.sequence.push_back(data.getVehicleEndNode(k));
        evaluator->evaluateRoute(r);
        emptySol.routes.push_back(r);
    }
    for (const auto& route : emptySol.routes) {
        addRouteToPool(route);
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
    bestFeasibleSolution = currentSol.hasViolations ? std::nullopt : std::make_optional(currentSol);

    std::unordered_set<std::size_t> visitedSolutionsHashes;
    visitedSolutionsHashes.insert(SolutionHash{}(currentSol));

    initializeStatsAndTemperature(currentSol);
    initializeRoutePool();

    // 2. Main Loop
    for (iteration = 0; ; ++iteration) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (stoppingCriteria(iteration, elapsed.count())) break;

        // Adaptive Operator Selection
        int destroyOpIdx = selectOperator(destroyStats.weights);
        int repairOpIdx = selectOperator(repairStats.weights);
        ALNSSolution neighbor = currentSol;

        // --- Destroy and Repair ---
        applyDestroy(neighbor, destroyOpIdx);
        applyRepair(neighbor, repairOpIdx);
        evaluator->evaluateSolution(neighbor);

        std::size_t neighborHash = SolutionHash{}(neighbor);
        bool isNew = (visitedSolutionsHashes.find(neighborHash) == visitedSolutionsHashes.end());
        if (isNew)  visitedSolutionsHashes.insert(neighborHash);

        for (const auto& route : neighbor.routes) {
            if (!route.sequence.empty()) {
                addRouteToPool(route);
            }
        }

        // --- Acceptance Criterion ---
        double iterScore = 0.0;
        bool accept = acceptanceCriteria(neighbor.objectiveValue, currentSol.objectiveValue, currentTemperature, isNew, iterScore);

        if (accept) {
            currentSol = neighbor;
            updateBestSolutions(neighbor);
        }
        
        // -- Update Operator Stats ---
        destroyStats.scores[destroyOpIdx] += iterScore;
        destroyStats.timesUsed[destroyOpIdx] += 1;
        repairStats.scores[repairOpIdx] += iterScore;
        repairStats.timesUsed[repairOpIdx] += 1;

        if (iteration > 0 && iteration % params->segmentIterations == 0) {
            updateWeights(destroyStats);
            updateWeights(repairStats);
        }

        // --- Cooling ---
        currentTemperature *= params->coolingRate;

        // --- Matheuristic Integration ---
        if (iteration > 0 && iteration % params->setPartitioningInterval == 0)
            solveMatheuristic();
    }

    // Final clean run of SP
    if (hybridMethod != HybridMethod::NONE) {
        logger.log("Final matheuristic run to polish solution...");
        solveMatheuristic();
    }

    // Solve the schedule later (only if the best solution has violations)
    if (bestSolution.hasViolations) {
        logger.log("Best solution has violations. Attempting to fix with Schedule Later (CPLEX)...");
        result = solveScheduleLater(bestSolution);
        // If it still has violations and a feasible solution exists, return the best feasible one found during the search
        if (result->solverStatus == "Semi-Feasible" && bestFeasibleSolution.has_value()) {
            logger.log(std::string("Schedule Later did not find a fully feasible solution. ") +
                        "Returning best feasible solution found during ALNS search " +
                        "with objective: " + std::to_string(bestFeasibleSolution->objectiveValue));
            result = bestFeasibleSolution->toResultInstance(data, this->solveTime);
        }
    }
    else {
        logger.log("Best solution is feasible. Preparing result instance...");
        result = bestSolution.toResultInstance(data, this->solveTime);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = end - start;
    this->solveTime = totalElapsed.count();
    result->solveTime = this->solveTime;



    // Print operator stats
    if (verbose) {
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
    }
}

void ALNSSolver::updateBestSolutions(const ALNSSolution& candidate, std::string context) {
    bool bestImproved = false;
    bool feasibleImproved = false;

    // Update global best solution if improved
    if (candidate.objectiveValue < bestObjective) {
        bestSolution = candidate;
        bestObjective = candidate.objectiveValue;
        bestImproved = true;
    }

    // Update best feasible solution if improved and has no violations
    if (!candidate.hasViolations) {
        if (!bestFeasibleSolution.has_value() || candidate.objectiveValue < bestFeasibleSolution->objectiveValue) {
            bestFeasibleSolution = candidate;
            feasibleImproved = true;
        }
    }

    // Logs
    if (bestImproved) {
        logger.log(context + "* Iter " + std::to_string(iteration) + ": New Best = " + std::to_string(candidate.objectiveValue) 
            + " (Violations: " + (candidate.hasViolations ? "Yes" : "No") + ")");
    }
    else if (feasibleImproved) {
        logger.log(context + "* Iter " + std::to_string(iteration) + ": New Best Feasible = " + std::to_string(candidate.objectiveValue));
    }
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
    if (result.has_value()) {
        return result.value();
    } else {
        throw std::runtime_error("Result is not available yet. Call solve() first.");
    }
}
