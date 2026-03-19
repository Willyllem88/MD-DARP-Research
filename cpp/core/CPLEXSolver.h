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

class CPLEXSolver: public Solver {
public:
    CPLEXSolver(
        DARPMD_ProblemInstance& instance,
        std::optional <double> timeLimit = std::nullopt
    );

    ~CPLEXSolver();

    void solve() override;

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

    // --- Auxiliary Sets ---
    // List of valid arcs (i,j,k)
    std::vector<std::tuple<int, int, int>> A_k;
    // All involved nodes (P u D u Starts u Ends)
    std::vector<int> V_nodes;

    // --- Variables ---
    // Key: <i, j, k>: whether vehicle k travels from i to j
    std::map<std::tuple<int, int, int>, IloNumVar> x;
    // Key: <node_id, vehicle_id>: time at which vehicle k arrives at node i
    std::map<int, IloNumVar> u;
    // Key: <node_id, vehicle_id>: load of vehicle k upon arrival at node i
    std::map<std::pair<int, int>, IloNumVar> w;

    // --- Methods ---
    void tightenTimeWindows();

    // Build the model (Variables, Objective, Constraints)
    void buildModel();
    // Check if an arc is feasible
    bool isArcFeasible(uint i, uint j, uint k) const;
    // Check if a path is feasible
    bool checkPathFeasibility(const std::vector<uint>& path, uint k) const;
    
    // Helper to check if a tuple exists in the map (like "if (i,j,k) in m.A_k")
    bool varExists(int i, int j, int k) const;

    // Solver statistics
    double objectiveValue;
    double solveTime;
    double mipGap;
};
