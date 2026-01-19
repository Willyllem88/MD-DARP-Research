#pragma once

#include "Solver.h"
#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"
#include <vector>
#include <map>
#include <random>
#include <optional>

// Represents a specific route for a vehicle
struct TabuRoute {
    int vehicleId;
    std::vector<int> sequence; // Sequence of nodes visited (including start/end depots)
    
    // Evaluation metrics (cached for performance)
    double durationViolation = 0.0;
    double loadViolation = 0.0;
    double timeWindowViolation = 0.0;
    double rideTimeViolation = 0.0;
    double routeCost = 0.0;
    
    // Timing information (calculated via the 8-step procedure)
    std::map<int, double> arrivalTimes;
    std::map<int, double> departureTimes;
    std::map<int, double> waitTimes;
    std::map<int, double> beginServiceTimes;
    std::map<int, double> rideTimes; // Key is request ID (pickup node)
};

// Represents a full solution
struct TabuSolution {
    std::vector<TabuRoute> routes;
    double costFunctionValue = 0.0; // f(s)
    double rawObjectiveValue = 0.0; // c(s)
    
    // Total violations
    double totalLoadV = 0.0;
    double totalDurationV = 0.0;
    double totalTWV = 0.0;
    double totalRideV = 0.0;
    
    bool isFeasible() const {
        return totalLoadV < 1e-4 && totalDurationV < 1e-4 && 
               totalTWV < 1e-4 && totalRideV < 1e-4;
    }
};

class DARPMDTabuSolver : public Solver {
private:
    const DARPMD_ProblemInstance& data;
    std::optional<double> timeLimit;
    
    // Algorithm Parameters (Cordeau & Laporte 2003)
    double alpha = 1.0; // Weight for Load
    double beta = 1.0;  // Weight for Duration
    double gamma = 1.0; // Weight for Time Windows
    double tau = 1.0;   // Weight for Ride Time
    
    int maxIterations = 10000;
    int tabuTenure = 10; 
    
    // Tabu Memory: tabuList[requestId][vehicleId] = iteration until forbidden
    std::vector<std::vector<int>> tabuList;
    
    // Diversification: freq[requestId][vehicleId] = count of assignments
    std::vector<std::vector<int>> freqMatrix;
    
    // Best solution found
    TabuSolution bestFeasibleSolution;
    bool feasibleFound = false;
    double finalSolveTime = 0.0;

    // Random engine
    std::mt19937 rng;

public:
    DARPMDTabuSolver(const DARPMD_ProblemInstance& instance, std::optional<double> timeLimit = std::nullopt);
    ~DARPMDTabuSolver() override = default;

    void solve() override;
    DARPMD_ResultInstance getResult() const override;
    std::string name() const override { return "Tabu Search (Cordeau & Laporte)"; }

private:
    // --- Core Logic Steps ---
    TabuSolution generateInitialSolution();
    
    // Evaluation using the 8-step forward time slack procedure [Section 4.7]
    void evaluateSolution(TabuSolution& sol);
    void evaluateRoute(TabuRoute& route); 
    
    // Neighbor operator: Move request i from route k to k' [Section 4.2]
    TabuSolution bestNeighbor(const TabuSolution& currentSol, int iter);
    
    // Helper to calculate f(s) with penalties
    double calculatePenalizedCost(const TabuSolution& sol);
    
    // Helper to calculate diversification penalty p(s) [Section 4.3]
    double calculateDiversificationPenalty(const TabuSolution& sol, int reqId, int vehicleId);

    // Helpers
    int getDeliveryNode(int pickupNode) const;
    bool isPickup(int node) const;
    bool isDelivery(int node) const;
};