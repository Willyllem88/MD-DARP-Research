#pragma once

#include "Solver.h"
#include "MDDARP_ProblemInstance.h"
#include "MDDARP_ResultInstance.h"
#include "alns/ALNSRoute.h"
#include "alns/ALNSSolution.h"
#include "alns/ALNSParams.h"
#include "alns/ALNSEvaluator.h"
#include "alns/SetBasedSolver.h"
#include "alns/SetPartitioningSolver.h"
#include "alns/SetCoveringSolver.h"
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
    enum class HybridMethod {NONE, SET_PARTITIONING, SET_COVERING};


    ALNSSolver(
        MDDARP_ProblemInstance& instance, 
        std::optional<double> timeLimit = std::nullopt, 
        HybridMethod hybridMethod = HybridMethod::NONE,
        int seed = 42, 
        bool verbose = false,
        const ALNSParams& params = ALNSParams(),
        bool enableNR = false
    );
        
    ~ALNSSolver() {};

    void solve() override;
    MDDARP_ResultInstance getResult() const override;

    // Returns a string with the name of the solver configuration (for 
    // logging purposes). In can be ALNS, ALNS_SP or ALNS_SC depending on 
    // the hybrid method used.
    std::string name() const override;

    // Utility to save a route to the pool if it's good/feasible
    void addRouteToPool(const ALNSRoute& route);

private:
    MDDARP_ProblemInstance& data;
    std::optional<MDDARP_ResultInstance> result;
    std::optional<double> timeLimit;

    HybridMethod hybridMethod;
    
    std::unique_ptr<ALNSParams> params;
    std::unique_ptr<ALNSEvaluator> evaluator;
    std::unique_ptr<SetBasedSolver> setSolver;
    std::unique_ptr<ALNSOperators> operators;

    // Random engine
    std::mt19937 rng;

    // Start time for time limit tracking
    std::chrono::steady_clock::time_point startTime;

    // ALNS iteration
    int iteration;

    // Current global status
    ALNSSolution bestSolution;
    double bestObjective;
    std::optional<ALNSSolution> bestFeasibleSolution;
    double solveTime;

    double currentTemperature;

    // --- The Route Pool for Set Partitioning ---
    // Stores unique feasible routes found during search
    // Key: VehicleID (since depots differ), Value: List of routes
    std::map<int, std::vector<ALNSRoute>> routePool;
    std::unordered_map<int, std::unordered_set<std::vector<int>, RouteSequenceHash>> seenRoutes;

    // --- Core Logic Methods ---

    bool stoppingCriteria();
    bool acceptanceCriteria(double candidateObj, double currentObj, double temperature, bool isNew, double& score);
    
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
    void applyDestroy(ALNSSolution& sol, int destroyOpIdx);
    void applyRepair(ALNSSolution& sol, int repairOpIdx);

    void updateBestSolutions(const ALNSSolution& candidate, std::string context = "");
    
    // Solution Management
    ALNSSolution createInitialSolution();

    void initializeStatsAndTemperature(const ALNSSolution& initialSolution);
    void initializeRoutePool(); // Initializes the route pool with empty routes for each vehicle

    // --- CPLEX Integration (Set Partitioning / Covering) ---
    // Solves a Set Partitioning/Covering problem using the accumulated routePool
    void solveMatheuristic(); 
    // Solve schedule later
    MDDARP_ResultInstance solveScheduleLater(ALNSSolution& sol);
};