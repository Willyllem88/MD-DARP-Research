#pragma once

#include <ilcplex/ilocplex.h>
#include <map>
#include <tuple>
#include <vector>
#include <utility>
#include <optional>

#include "Solver.h"
#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"

class CPLEXSoftSolver: public Solver {
public:
    CPLEXSoftSolver(
        DARPMD_ProblemInstance& instance,
        std::optional <double> timeLimit = std::nullopt,
        bool verbose = false
    );

    ~CPLEXSoftSolver();

    void solve() override;

    // For analysis: solve the LP relaxation of the model (i.e., relax integrality constraints)
    // This can be useful to understand the strength of the formulation and the quality of the LP bound.
    double solveLPRelaxation();

    // For debugging and analysis, return the number of constraints and variables in the model
    // before CPLEX preprocessing (i.e., the original model size)
    int getNumberOfConstraints() const;
    int getNumberOfVariables() const;

    DARPMD_ResultInstance getResult() const override;

    std::string name() const override {
        return "ILP (CPLEX)";
    }

private:
    DARPMD_ProblemInstance& data;

    std::optional<double> timeLimit;
    
    // CPLEX Environment and Model
    IloEnv env;
    IloModel model;
    IloCplex cplex;

    // Violaton multipliers
    const double alpha = 10000.0; // For load violations
    const double beta  = 10000.0; // For duration violations
    const double gamma = 10000.0; // For ride time window violations
    const double tau   = 10000.0; // For time ride time violations

    // --- Auxiliary Sets ---
    // List of valid arcs (i,j,k)
    std::vector<std::tuple<int, int, int>> A_k;
    // All involved nodes (P u D u Starts u Ends)
    std::vector<int> V_nodes;

    // --- Variables ---
    // Key: <i, j, k>: whether vehicle k travels from i to j
    std::map<std::tuple<int, int, int>, IloNumVar> x;
    // Key: <node_id>: time at which a vehicle (only one will arrive due to
    // constraints) arrives at node i
    std::map<int, IloNumVar> u;
    // Key: <node_id, vehicle_id>: load of vehicle k upon arrival at node i
    std::map<std::pair<int, int>, IloNumVar> w;
    // Soft constraint violation variables
    std::map<std::pair<int,int>, IloNumVar> viol_load;
    std::map<int, IloNumVar> viol_duration;
    std::map<int, IloNumVar> viol_tw;
    std::map<int, IloNumVar> viol_ridetime;

    // Build the model (Variables, Objective, Constraints)
    void buildModel();
    
    // Helper to check if a tuple exists in the map (like "if (i,j,k) in m.A_k")
    bool varExists(int i, int j, int k) const;

    // Solver statistics
    double objectiveValue;
    double solveTime;
    double mipGap;
};
