#pragma once

#include <ilcplex/ilocplex.h>
#include <map>
#include <tuple>
#include <vector>
#include <utility>


#include "DARPMD_ProblemInstance.h"
#include "DARPMD_ResultInstance.h"

class DARPMDSolver {
public:
    DARPMDSolver(const DARPMD_ProblemInstance& instance);
    ~DARPMDSolver();

    void solve(double time_limit_sec = 3600.0);

    void displayResults();

    DARPMD_ResultInstance extractResult();

private:
    const DARPMD_ProblemInstance& data;
    
    // CPLEX Environment and Model
    IloEnv env;
    IloModel model;
    IloCplex cplex;

    // --- Auxiliary Sets ---
    // List of valid arcs (i,j,k) calculated exactly like the Python valid_arcs
    std::vector<std::tuple<int, int, int>> A_k;
    // All involved nodes (P u D u Starts u Ends)
    std::vector<int> V_nodes;

    // --- Variables ---
    // Key: <i, j, k>: whether vehicle k travels from i to j
    std::map<std::tuple<int, int, int>, IloNumVar> x;
    // Key: <node_id, vehicle_id>: time at which vehicle k arrives at node i
    std::map<std::pair<int, int>, IloNumVar> u;
    // Key: <node_id, vehicle_id>: load of vehicle k upon arrival at node i
    std::map<std::pair<int, int>, IloNumVar> w;

    // Build the model (Variables, Objective, Constraints)
    void buildModel();
    
    // Helper to check if a tuple exists in the map (like "if (i,j,k) in m.A_k")
    bool varExists(int i, int j, int k) const;

    // Helper for Big-M
    const double M_time = 10000.0;
    const double M_load = 1000.0;
};