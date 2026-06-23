#pragma once

#include <ilcplex/ilocplex.h>
#include <map>
#include <tuple>
#include <vector>
#include <utility>
#include <optional>

#include "Solver.h"
#include "MDDARP_ProblemInstance.h"
#include "MDDARP_ResultInstance.h"

class CPLEXSolver: public Solver {
public:
    CPLEXSolver(
        MDDARP_ProblemInstance& instance,
        std::optional <double> timeLimit = std::nullopt,
        bool verbose = false
    );

    ~CPLEXSolver();

    void solve() override;

    // For analysis: solve the LP relaxation of the model (i.e., relax integrality constraints)
    // This can be useful to understand the strength of the formulation and the quality of the LP bound.
    double solveLPRelaxation();

    // For debugging and analysis, return the number of constraints and variables in the model
    // before CPLEX preprocessing (i.e., the original model size)
    int getNumberOfConstraints() const;
    int getNumberOfVariables() const;

    // After solving, extract the solution and return it in structured and uniform format
    MDDARP_ResultInstance getResult() const override;

    std::string name() const override {
        return "ILP (CPLEX)";
    }

private:
    MDDARP_ProblemInstance& data;

    std::optional<double> timeLimit;
    
    // CPLEX Environment and Model
    IloEnv env;
    IloModel model;
    IloCplex cplex;

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

    // --- Methods ---
    void tightenTimeWindows();

    // Build the model (Variables, Objective, Constraints)
    void buildModel();

    // Check if an arc is feasible: that means, if a feasible solution exists, this 
    // arc could be used in such solution. This prunes the number of arcs and reduces
    // the size of the model, which is important for performance.
    bool isArcFeasible(uint i, uint j, uint k) const;

    // Check if a path (e.g., a sequence of nodes) is feasible regarding time windows,
    // vehicle capacity, and ride time constraints. This is useful to prune arcs.
    bool checkPathFeasibility(const std::vector<uint>& path, uint k) const;
    
    // Helper to check if a tuple exists in the map (like "if (i,j,k) in m.A_k")
    bool varExists(int i, int j, int k) const;

    // Solver statistics
    double objectiveValue;
    double solveTime;
    double mipGap;
};
