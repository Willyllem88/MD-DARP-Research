#pragma once

#include "Solver.h"
#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"
#include "alns/ALNSRoute.h"
#include "alns/ALNSSolution.h"
#include "alns/ALNSParams.h"
#include "alns/ALNSEvaluator.h"
#include "alns/SetPartitioningSolver.h"
#include "alns/ALNSOperators.h"

#include <ilcplex/ilocplex.h>

#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <string>
#include <random>
#include <optional>

class ALNSEvaluator; // Forward declaration to avoid circular dependency

class ALNSSolver : public Solver {
public:
    ALNSSolver(
        DARPMD_ProblemInstance& instance, 
        std::optional<double> timeLimit = std::nullopt, 
        int seed = 42, 
        bool verbose = false);
        
    ~ALNSSolver();

    void solve() override;
    DARPMD_ResultInstance getResult() const override;

    std::string name() const override {
        return "Matheuristic (ALNS + CPLEX Set Partitioning)";
    }

    // Utility to save a route to the pool if it's good/feasible
    // TODO: this function will be declared in anothar place
    void addRouteToPool(const ALNSRoute& route);

private:
    DARPMD_ProblemInstance& data;
    std::optional<DARPMD_ResultInstance> result;
    std::optional<double> timeLimit;
    
    std::unique_ptr<ALNSParams> params;
    std::unique_ptr<ALNSEvaluator> evaluator;
    std::unique_ptr<SetPartitioningSolver> spSolver;
    std::unique_ptr<ALNSOperators> operators;

    // Random engine
    std::mt19937 rng;

    // Current global status
    ALNSSolution bestSolution;
    double bestObjective;
    double solveTime;

    double currentTemperature;

    // --- The Route Pool for Set Partitioning ---
    // Stores unique feasible routes found during search
    // Key: VehicleID (since depots differ), Value: List of routes
    std::map<int, std::vector<ALNSRoute>> routePool;
    std::unordered_map<int, std::unordered_set<std::vector<int>, RouteSequenceHash>> seenRoutes;

    // --- Core Logic Methods ---

    bool stoppingCriteria(int iter, double elapsedSeconds);
    bool acceptanceCriteria(double candidateObj, double currentObj, double temperature, bool isNew, double& score);

    void applyDestroy(ALNSSolution& sol, int destroyOpIdx);
    void applyRepair(ALNSSolution& sol, int repairOpIdx);
    
    // Solution Management
    ALNSSolution createInitialSolution();

    enum class DestroyMethod {RANDOM, WORST, SHAW, COUNT}; // COUNT the number of methods for stats
    enum class RepairMethod {GREEDY, REGRET2, COUNT};

    struct OperatorStats {
        std::vector<double> weights;
        std::vector<int> scores;
        std::vector<int> timesUsed;

        void init(int size) {
            weights.assign(size, 1.0); // All operators start with equal weight
            scores.assign(size, 0.0);
            timesUsed.assign(size, 0);
        }
    };
    OperatorStats destroyStats;
    OperatorStats repairStats;

    int selectOperator(const std::vector<double>& weights);
    void updateWeights(OperatorStats& stats);
    void initializeStatsAndTemperature(const ALNSSolution& initialSolution);

    // Helper: Check if any delivery appears before its pickup in the solution (should never happen)
    void checkPickupAfterDelivery(const ALNSSolution& sol, const DARPMD_ProblemInstance& data) const;

    // --- CPLEX Integration (Set Partitioning) ---
    // Solves a Set Partitioning problem using the accumulated routePool
    void solveSetPartitioning(); 
    // Solve schedule later
    DARPMD_ResultInstance solveScheduleLater(ALNSSolution& sol);

    // TODO: future improvements
    void runSimpleLocalSearch(ALNSSolution& sol);
};