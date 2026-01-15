#include "DARPMDSolver.h"
#include <iostream>
#include <algorithm>
#include <set>

DARPMDSolver::DARPMDSolver(const DARPMD_ProblemInstance& instance) 
    : data(instance), model(env), cplex(model) {
    
    // 1. Initialize Sets
    
    // Create Set V: P + D + StartNodes + EndNodes
    std::set<int> distinct_nodes;
    for (int i : data.P) distinct_nodes.insert(i);
    for (int i : data.D) distinct_nodes.insert(i);
    for (auto const& [k, node] : data.StartNode) distinct_nodes.insert(node);
    for (auto const& [k, node] : data.EndNode) distinct_nodes.insert(node);
    
    V_nodes.assign(distinct_nodes.begin(), distinct_nodes.end());

    // Create Set A_k: Valid Arcs
    for (int k : data.K) {
        int sk = data.StartNode.at(k);
        int ek = data.EndNode.at(k);

        // nodes_k = P u D + [sk, ek]
        std::vector<int> nodes_k;
        nodes_k.insert(nodes_k.end(), data.P.begin(), data.P.end());
        nodes_k.insert(nodes_k.end(), data.D.begin(), data.D.end());
        nodes_k.push_back(sk);
        nodes_k.push_back(ek);

        for (int i : nodes_k) {
            for (int j : nodes_k) {
                if (i == j) continue;
                if (i == ek) continue; // Does not leave the end
                if (j == sk) continue; // Does not enter the start                
                A_k.emplace_back(i, j, k);
            }
        }
    }

    buildModel();
}

DARPMDSolver::~DARPMDSolver() {
    env.end();
}

bool DARPMDSolver::varExists(int i, int j, int k) const {
    return x.find({i, j, k}) != x.end();
}

void DARPMDSolver::buildModel() {
    std::cout << "Building CPLEX Model..." << std::endl;

    // --- 1. Create Variables ---

    // x[i, j, k] - Binary
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        std::string name = "x_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
        x[{i, j, k}] = IloNumVar(env, 0, 1, ILOBOOL, name.c_str());
    }

    // u[i, k] - Continuous (Time) and w[i, k] - Continuous (Load)
    // Defined for all nodes in V and vehicles K, but usually we only need them where valid.
    // To match Pyomo "m.V, m.K", we iterate all combinations.
    for (int i : V_nodes) {
        for (int k : data.K) {
            std::string u_name = "u_" + std::to_string(i) + "_" + std::to_string(k);
            std::string w_name = "w_" + std::to_string(i) + "_" + std::to_string(k);
            
            u[{i, k}] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, u_name.c_str());
            w[{i, k}] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, w_name.c_str());
        }
    }

    // --- 2. Objective Function ---

    // Minimize sum(c_ijk * x_ijk)
    IloExpr objExpr(env);
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        double cost = data.getCost(i, j, k);
        objExpr += cost * x[{i, j, k}];
    }
    model.add(IloMinimize(env, objExpr));
    objExpr.end();

    // --- 3. Constraints ---

    // Each request served once (c1)
    for (int i : data.P) {
        IloExpr expr(env);
        for (int k : data.K) {
            // Find valid j for this i, k
            for (const auto& [ii, jj, kk] : A_k) {
                if (ii == i && kk == k) {
                    expr += x[{ii, jj, kk}];
                }
            }
        }
        model.add(expr == 1);
        expr.end();
    }

    // Flow conservation (c2)
    std::vector<int> PuD = data.P;
    PuD.insert(PuD.end(), data.D.begin(), data.D.end());

    for (int i : PuD) {
        for (int k : data.K) {
            IloExpr inFlow(env);
            IloExpr outFlow(env);
            
            // Build sums based on valid arcs in A_k
            for (const auto& [ii, jj, kk] : A_k) {
                if (kk == k) {
                    if (jj == i) inFlow += x[{ii, jj, kk}];
                    if (ii == i) outFlow += x[{ii, jj, kk}];
                }
            }

            model.add(inFlow - outFlow == 0);
            inFlow.end();
            outFlow.end();
        }
    }

    // Flow at Depots (c3)
    for (int k : data.K) {
        // Start Depot Rule
        int sk = data.StartNode.at(k);
        IloExpr startExpr(env);
        for (const auto& [ii, jj, kk] : A_k) {
            if (ii == sk && kk == k) {
                startExpr += x[{ii, jj, kk}];
            }
        }
        model.add(startExpr == 1);
        startExpr.end();

        // End Depot Rule
        int ek = data.EndNode.at(k);
        IloExpr endExpr(env);
        for (const auto& [ii, jj, kk] : A_k) {
            if (jj == ek && kk == k) {
                endExpr += x[{ii, jj, kk}];
            }
        }
        model.add(endExpr == 1);
        endExpr.end();
    }

    // Pairing (c4)
    for (int i : data.P) {
        int delivery_node = i + data.N_requests;
        for (int k : data.K) {
            IloExpr flowPick(env);
            IloExpr flowDel(env);
            
            for (const auto& [ii, jj, kk] : A_k) {
                if (kk == k) {
                    if (ii == i) flowPick += x[{ii, jj, kk}];
                    if (ii == delivery_node) flowDel += x[{ii, jj, kk}];
                }
            }
            model.add(flowPick - flowDel == 0);
            flowPick.end();
            flowDel.end();
        }
    }

    // c5: Time Consistency
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        const double M = 10000.0;

        double serv = data.getServiceTime(i);
        double trav = data.getTravelTime(i, j);
        
        model.add(
            u[{j, k}] >= u[{i, k}] + serv + trav - M * (1 - x[arc])
        );
    }

    // c6: Time Windows
    for (int i : V_nodes) {
        for (int k : data.K) {
            // Check limits. If vector indices match node IDs
            double ei = data.getTimeWindowStart(i);
            double li = data.getTimeWindowEnd(i);
            
            model.add(u[{i, k}] >= ei);
            model.add(u[{i, k}] <= li);
        }
    }

    // c7: Max Ride Time (Actually Max Route Duration T_max)
    for (int k : data.K) {
        int sk = data.StartNode.at(k);
        int ek = data.EndNode.at(k);
        double Tmax = data.max_route_time.at(k);
        
        model.add(u[{ek, k}] - u[{sk, k}] <= Tmax);
    }

    // c8: Precedence
    for (int i : data.P) {
        const double M = 10000.0;

        int delivery_node = i + data.N_requests;
        double serv = data.getServiceTime(i);
        double trav = data.getTravelTime(i, delivery_node);
        
        for (int k : data.K) {
            IloExpr k_serves_i(env);
            for (const auto& arc : A_k) {
                if (std::get<0>(arc) == i && std::get<2>(arc) == k) {
                    k_serves_i += x[arc];
                }
            }

            model.add(
                u[{delivery_node, k}] >= u[{i, k}] + serv + trav - M * (1 - k_serves_i)
            );
            k_serves_i.end();
        }
    }

    //TODO: check from now on
    // c9: Ride Time Limit (L)
    // u[del, k] - (u[pick, k] + serv) <= L + M(1 - k_serves_i)
    for (int i : data.P) {
        int delivery_node = i + data.N_requests;
        double serv = data.getServiceTime(i);
        double L = data.max_ride_time;

        for (int k : data.K) {
            IloExpr k_serves_i(env);
            for (const auto& arc : A_k) {
                if (std::get<0>(arc) == i && std::get<2>(arc) == k) {
                    k_serves_i += x[arc];
                }
            }

            IloExpr lhs(env);
            lhs += u[{delivery_node, k}];
            lhs -= u[{i, k}];
            lhs -= serv;

            IloExpr rhs(env);
            rhs += L;
            rhs += M_time;
            rhs -= M_time * k_serves_i;

            model.add(lhs <= rhs);
            
            lhs.end();
            rhs.end();
            k_serves_i.end();
        }
    }

    // c10: Load Consistency
    // w[j,k] >= w[i,k] + q[j] - M(1 - x)
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        double q_j = data.getDemand(j);
        
        IloExpr lhs(env);
        lhs += w[{j, k}];
        lhs -= w[{i, k}];
        lhs -= M_load * x[arc];
        
        model.add(lhs >= q_j - M_load);
        lhs.end();
    }

    // c11: Load Feasible Bounds
    for (int i : V_nodes) {
        double qi = data.getDemand(i);
        for (int k : data.K) {
            double Qk = data.capacity.at(k);
            double lb = std::max(0.0, qi);
            double ub = std::min(Qk, Qk + qi);
            
            model.add(w[{i, k}] >= lb);
            model.add(w[{i, k}] <= ub);
        }
    }

    // c12: Depot Initial/Final Load
    for (int k : data.K) {
        int sk = data.StartNode.at(k);
        int ek = data.EndNode.at(k);
        model.add(w[{sk, k}] == 0);
        model.add(w[{ek, k}] == 0);
    }
}

void DARPMDSolver::solve(std::optional <double> timeLimit) {
    // Set time limit if provided
    if (timeLimit.has_value()) {
        cplex.setParam(IloCplex::Param::TimeLimit, timeLimit.value());
    }
    
    std::cout << "Starting CPLEX solve..." << std::endl;
    if (cplex.solve()) {
        std::cout << "CPLEX Status: " << cplex.getStatus() << std::endl;
        std::cout << "Objective Value: " << cplex.getObjValue() << std::endl;
    } else {
        std::cout << "No solution found or infeasible. Status: " << cplex.getStatus() << std::endl;
    }
}

DARPMD_ResultInstance DARPMDSolver::extractResult() {
    DARPMD_ResultInstance result(data);

    // 1. General Solution Info
    try {
        result.objectiveValue = cplex.getObjValue();
        result.solverStatus = (cplex.getStatus() == IloAlgorithm::Optimal) ? "Optimal" : "Feasible";
    } catch (...) {
        result.solverStatus = "No Solution";
        return result;
    }

    // 2. Reconstruct routes (Logic similar to displayResults but storing data)
    for (int k : data.K) {
        VehicleRoute vRoute;
        vRoute.vehicleId = k;

        // Map of next nodes for this vehicle
        std::map<int, int> next_node_map;
        for (const auto& arc : A_k) {
            auto [i, j, veh] = arc;
            if (veh == k) {
                // Check if variable is 1 (with numerical tolerance)
                if (cplex.isExtracted(x[arc]) && cplex.getValue(x[arc]) > 0.5) {
                    next_node_map[i] = j;
                }
            }
        }

        int current_node = data.StartNode.at(k);
        int end_node = data.EndNode.at(k);

        // If vehicle doesn't move
        if (next_node_map.find(current_node) == next_node_map.end()) {
            // Still, we can add the start node for consistency
            RouteStep startStep;
            startStep.nodeId = current_node;
            startStep.type = "DepotStart";
            startStep.arrivalTime = 0.0;
            startStep.loadAfter = 0.0;
            vRoute.steps.push_back(startStep);
            
            result.addRoute(k, vRoute);
            continue;
        }

        // Traverse the route
        while (true) {
            // Create the current step
            RouteStep step;
            step.nodeId = current_node;
            
            // Determine type (simple heuristic based on your sets P and D)
            if (current_node == data.StartNode.at(k)) step.type = "DepotStart";
            else if (current_node == data.EndNode.at(k)) step.type = "DepotEnd";
            else if (std::find(data.P.begin(), data.P.end(), current_node) != data.P.end()) step.type = "Pickup";
            else if (std::find(data.D.begin(), data.D.end(), current_node) != data.D.end()) step.type = "Delivery";
            else step.type = "Node";

            // Extract continuous variable values u (time) and w (load)
            // Note: Handle with try-catch or checks if variable exists for that node/vehicle
            if (u.count({current_node, k})) 
                step.arrivalTime = cplex.getValue(u[{current_node, k}]);
            else 
                step.arrivalTime = 0.0;

            if (w.count({current_node, k}))
                step.loadAfter = cplex.getValue(w[{current_node, k}]);
            else 
                step.loadAfter = 0.0;

            vRoute.steps.push_back(step);

            // Stopping condition
            if (current_node == end_node) break;
            if (next_node_map.find(current_node) == next_node_map.end()) break; // Broken route?

            // Advance
            current_node = next_node_map[current_node];
        }
        
        result.addRoute(k, vRoute);
    }

    return result;
}