#pragma once

#include "Solver.h"
#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"

#include <ilcplex/ilocplex.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <string>
#include <random>
#include <optional>

// Configuration for the ALNS
struct ALNSParams {
    int maxIterations = 2000;
    int setPartitioningInterval = 250; // Run CPLEX SP every X iterations
    double initialTemperature = 100.0;
    double coolingRate = 0.9995;
    double destroyFraction = 0.4; // Fraction of requests to remove in destroy phase
    double worstRemovalPower = 3.0; // For destroyWorst
    
    // Penalties
    double timeWindowPenalty = 100.0;           // per minute
    double vehicleMaxRouteTimePenalty = 100.0;  // per minute
    double capacityPenalty = 100.0;             // per unit
    double rideTimePenalty = 100.0;             // per minute
    double unassignedPenalty = 100000.0; // per request

    // Punction for adaptive operator selection constants
    const double sigma1 = 33.0; // For new best global
    const double sigma2 = 9.0;  // For better than current
    const double sigma3 = 13.0;  // For accepted (but not better)
    const double reactionFactor = 0.1; // How much to adjust weights based on performance
};

// Internal representation of a single vehicle route
struct ALNSRoute {
    int vehicleId;
    std::vector<int> sequence; // Sequence of Node IDs (Depot -> ... -> Depot)
    
    // Evaluation metrics
    double distanceCost = 0.0;
    double timeWindowViolation = 0.0;
    double vehicleMaxRouteTimeViolation = 0.0;
    double loadViolation = 0.0;
    double rideTimeViolation = 0.0;
    double totalCost = 0.0; // penalized cost
    
    bool isFeasible = false;

    // Timestamps and loads for reconstruction
    std::map<int, double> arrivalTimes;
    std::map<int, double> loads;
};

struct RouteSequenceHash {
    std::size_t operator()(const std::vector<int>& seq) const {
        std::size_t hash = 0;
        for (int nodeId : seq) {
            hash ^= std::hash<int>{}(nodeId) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

// Internal representation of a full solution
struct ALNSSolution {
    std::vector<ALNSRoute> routes; // One per vehicle
    std::set<int> unassignedRequests; // IDs of requests in P not served
    double objectiveValue = 0.0; // Total penalized cost
};

class ALNSSolver : public Solver {
public:
    ALNSSolver(const DARPMD_ProblemInstance& instance, std::optional<double> timeLimit = std::nullopt);
    ~ALNSSolver();

    void solve() override;
    DARPMD_ResultInstance getResult() const override;

    std::string name() const override {
        return "Matheuristic (ALNS + CPLEX Set Partitioning)";
    }

private:
    const DARPMD_ProblemInstance& data;
    std::optional<double> timeLimit;
    ALNSParams params;

    // Random engine
    std::mt19937 rng;

    // Current global status
    ALNSSolution bestSolution;
    double bestObjective;
    double solveTime;

    // --- The Route Pool for Set Partitioning ---
    // Stores unique feasible routes found during search
    // Key: VehicleID (since depots differ), Value: List of routes
    std::map<int, std::vector<ALNSRoute>> routePool;
    std::unordered_map<int, std::unordered_set<std::vector<int>, RouteSequenceHash>> seenRoutes;

    // --- Core Logic Methods ---
    
    // Solution Management
    void evaluateRoute(ALNSRoute& route);
    void evaluateSolution(ALNSSolution& sol);
    ALNSSolution createInitialSolution();
    
    // ALNS Operators
    void destroyRandom(ALNSSolution& sol, int q);
    void destroyWorst(ALNSSolution& sol, int q);
    void destroyShaw(ALNSSolution& sol, int q);
    
    void repairGreedy(ALNSSolution& sol);
    void repairRegret2(ALNSSolution& sol);

    enum class DestroyMethod {RANDOM, WORST, SHAW, COUNT}; // COUNT the number of methods for stats
    enum class RepairMethod {GREEDY, REGRET2, COUNT};

    struct OperatorStats {
        std::vector<double> weights;
        std::vector<int> scores;
        std::vector<int> timesUsed;

        void init(int size) {
            weights.assign(size, 1.0); // Todos empiezan con igual probabilidad
            scores.assign(size, 0.0);
            timesUsed.assign(size, 0);
        }
    };
    OperatorStats destroyStats;
    OperatorStats repairStats;

    int selectOperator(const std::vector<double>& weights);
    void updateWeights(OperatorStats& stats);

    // Helper: Check if a request (pickup p, delivery d) can be inserted into route at pos i, j
    // Returns incremental cost (or infinity if impossible/too expensive)
    double calculateInsertionCost(const ALNSRoute& route, int requestIdx, int pIdx, int dIdx);
    
    // Helper: Calculate relatedness between two requests for adaptive selection in destroy
    double calculateRelatedness(int i, int j);

    // Helper: Check if solution has any violations (time windows, capacity, ride time)
    bool solutionHasViolations(const ALNSSolution& sol) const;

    // Helper: Check if solution has any violations (time windows, capacity, ride time) for debugging/logging
    void printSolutionDetails(const ALNSSolution& sol) const;

    // --- CPLEX Integration (Set Partitioning) ---
    // Solves a Set Partitioning problem using the accumulated routePool
    void solveSetPartitioning(); 

    // Utility to save a route to the pool if it's good/feasible
    void addRouteToPool(const ALNSRoute& route);
};