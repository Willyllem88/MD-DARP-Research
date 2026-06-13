#include "CPLEXSoftSolver.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <chrono>

CPLEXSoftSolver::CPLEXSoftSolver(DARPMD_ProblemInstance& instance, std::optional<double> timeLimit, bool verbose) 
    : Solver(verbose), data(instance), timeLimit(timeLimit), model(env), cplex(model) {

    // Mute CPLEX output if not verbose
    if (!verbose) {
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());
        cplex.setError(env.getNullStream());
    }
        
    // Create Set V: P u D u StartNodes u EndNodes
    std::set<int> distinct_nodes;
    for (int p : data.P) distinct_nodes.insert(p);
    for (int d : data.D) distinct_nodes.insert(d);
    for (int s : data.S) distinct_nodes.insert(s);
    for (int e : data.E) distinct_nodes.insert(e);

    V_nodes.assign(distinct_nodes.begin(), distinct_nodes.end());

    // Create Set A_k: Valid Arcs
    for (int k : data.K) {
        int sk = data.getVehicleStartNode(k);
        int ek = data.getVehicleEndNode(k);

        // nodes_k = P u D u [sk, ek]
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
                if (data.isVehicleStart(i) && data.isDelivery(j)) continue; // No direct start-delivery
                if (data.isPickup(i) && data.isVehicleEnd(j)) continue;     // No direct pickup-end
                if (data.isDelivery(i) && i == j + data.N_requests) continue; // No direct delivery-pickup of same request

                A_k.emplace_back(i, j, k);
            }
        }
    }

    buildModel();

    uint nb_a_k = A_k.size();
    uint total_possible_arcs = V_nodes.size() * V_nodes.size() * data.K.size();
    double ratio_pruned = 1.0 - ((double)nb_a_k / total_possible_arcs);
    logger.log("Total arcs generated (A_k): " + std::to_string(nb_a_k));
    logger.log("Total possible arcs: " + std::to_string(total_possible_arcs));
    logger.log("Ratio pruned: " + std::to_string(ratio_pruned * 100) + "%");
    logger.log("Number of variables: " + std::to_string(cplex.getNcols()));
    logger.log("Number of constraints: " + std::to_string(cplex.getNrows()));
    logger.log("\n\n");
}

CPLEXSoftSolver::~CPLEXSoftSolver() {
    env.end();
}

void CPLEXSoftSolver::solve() {
    // Set time limit if provided
    if (timeLimit.has_value()) {
        cplex.setParam(IloCplex::Param::TimeLimit, timeLimit.value());
    }
    
    logger.log("Starting CPLEX solve");

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = cplex.solve();
    auto end =  std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = end - start;
    this->solveTime = elapsed.count();

    // Print solve results
    if (solved) {
        logger.log("CPLEX Status: " + std::to_string(cplex.getStatus()));
        logger.log("Objective Value: " + std::to_string(cplex.getObjValue()));
        this->mipGap = cplex.getMIPRelativeGap();
        this->objectiveValue = cplex.getObjValue();
    } else {
        logger.log("No solution found or infeasible. Status: " + std::to_string(cplex.getStatus()));
    }
    logger.log("Total Solve Time: " + std::to_string(this->solveTime) + " s");
}

double CPLEXSoftSolver::solveLPRelaxation() {
    IloNumVarArray binaryVars(env);
    for (const auto& arc : A_k) {
        binaryVars.add(x[arc]); 
    }

    IloConversion lpRelaxation(env, binaryVars, ILOFLOAT);

    model.add(lpRelaxation);

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = cplex.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = end - start;

    double lpObjValue = cplex.getObjValue();

    if (solved) {
        logger.log("Status of the LP relaxation: " + std::to_string(cplex.getStatus()));
        logger.log("LP Objective Value (Lower Bound): " + std::to_string(lpObjValue));
    } else {
        logger.log("No solution found or infeasible. Status: " + std::to_string(cplex.getStatus()));
    }
    logger.log("LP Solving Time: " + std::to_string(elapsed.count()) + " s\n");

    model.remove(lpRelaxation);
    lpRelaxation.end();
    binaryVars.end();

    return lpObjValue;
}


int CPLEXSoftSolver::getNumberOfConstraints() const {
    return cplex.getNrows();
}

int CPLEXSoftSolver::getNumberOfVariables() const {
    return cplex.getNcols();
}

void CPLEXSoftSolver::fixAllRoutingVariablesToZero() {
    for (const auto& arc : A_k) {
        x[arc].setBounds(0.0, 0.0);
    }
}

void CPLEXSoftSolver::fixRoutingVariable(int i, int j, int k, double value) {
    if (varExists(i, j, k)) {
        x[{i, j, k}].setBounds(value, value);
    } else {
        std::cerr << "Warning: Attempting to fix non-existent variable x[" << i << "," << j << "," << k << "]\n";
    }
}

void CPLEXSoftSolver::unfixAllRoutingVariables() {
    for (const auto& arc : A_k) {
        x[arc].setBounds(0.0, 1.0);
    }
}

DARPMD_ResultInstance CPLEXSoftSolver::getResult() const {
    DARPMD_ResultInstance result(data); 

    // 1. General Solution Info
    try {
        result.objectiveValue = this->objectiveValue;
        result.solveTime = this->solveTime;
        result.mipGap = this->mipGap;

        double totalLoadViolation = 0.0;
        double totalDurationViolation = 0.0;
        double totalTWViolation = 0.0;
        double totalRideTimeViolation = 0.0;
        for (const auto& [key, var] : viol_load) {
            totalLoadViolation += cplex.getValue(var);
        }
        for (const auto& [key, var] : viol_duration) {
            totalDurationViolation += cplex.getValue(var);
        }
        for (const auto& [key, var] : viol_tw) {
            totalTWViolation += cplex.getValue(var);
        }
        for (const auto& [key, var] : viol_ridetime) {
            totalRideTimeViolation += cplex.getValue(var);
        }

        // Check status
        if (cplex.getStatus() == IloAlgorithm::Optimal) {
            if (totalLoadViolation > 1e-6 || totalDurationViolation > 1e-6 || totalTWViolation > 1e-6 || totalRideTimeViolation > 1e-6) {
                result.solverStatus = "Semi-Feasible";
            } else {
                result.solverStatus = "Optimal";
            }
        }
        else if (cplex.getStatus() == IloAlgorithm::Feasible) {
            result.solverStatus = "Feasible";
        }
        else {
            result.solverStatus = "No Solution";
        }
    } catch (...) {
        result.solverStatus = "No Solution";
        return result;
    }

    // 2. Reconstruct routes
    for (int k : data.K) {
        VehicleRoute vRoute;
        vRoute.vehicleId = k;

        // Map of next nodes for this vehicle
        std::map<int, int> next_node_map;
        for (const auto& arc : A_k) {
            auto [i, j, veh] = arc;
            if (veh == k) {
                // Check if variable is 1 (with numerical tolerance)
                if (cplex.isExtracted(x.at(arc)) && cplex.getValue(x.at(arc)) > 0.5) {
                    next_node_map[i] = j;
                }
            }
        }

        int current_node = data.getVehicleStartNode(k);
        int end_node = data.getVehicleEndNode(k);

        // If vehicle doesn't move
        if (next_node_map.find(current_node) == next_node_map.end()) {
            // Still, we can add the start node for consistency
            RouteStep startStep;
            startStep.nodeId = current_node;
            startStep.type = "DepotStart";
            startStep.beginServiceTime = 0.0;
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
            if (current_node == data.getVehicleStartNode(k)) step.type = "DepotStart";
            else if (current_node == data.getVehicleEndNode(k)) step.type = "DepotEnd";
            else if (std::find(data.P.begin(), data.P.end(), current_node) != data.P.end()) step.type = "Pickup";
            else if (std::find(data.D.begin(), data.D.end(), current_node) != data.D.end()) step.type = "Delivery";
            else step.type = "Node";

            // Extract continuous variable values u (time) and w (load)
            if (u.count(current_node))
                step.beginServiceTime = cplex.getValue(u.at(current_node));
            else
                step.beginServiceTime = 0.0;

            if (w.count({current_node, k}))
                step.loadAfter = cplex.getValue(w.at({current_node, k}));
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
    result.calculateViolations();

    return result;
}

void CPLEXSoftSolver::buildModel() {
    logger.log("Building CPLEX Model");

    // --- 1. Create Variables ---

    // x[i, j, k] - Binary
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        std::string name = "x_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
        x[{i, j, k}] = IloNumVar(env, 0, 1, ILOBOOL, name.c_str());
    }

    // u[i, k] - Continuous (Time) and w[i, k] - Continuous (Load)
    for (int i : V_nodes) {
        std::string u_name = "u_" + std::to_string(i);
        u[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, u_name.c_str());

        for (int k : data.K) {
            std::string w_name = "w_" + std::to_string(i) + "_" + std::to_string(k);
            w[{i, k}] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, w_name.c_str());
        }
    }

    // Violation Variables
    for (int k : data.K) {
        std::string dur_name = "viol_duration_" + std::to_string(k);
        viol_duration[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, dur_name.c_str());
    }
    for (int i : V_nodes) {
        std::string tw_name = "viol_tw_" + std::to_string(i);
        viol_tw[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, tw_name.c_str());

        for (int k : data.K) {
            std::string load_name = "viol_load_" + std::to_string(i) + "_" + std::to_string(k);
            viol_load[{i, k}] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, load_name.c_str());
        }

        if (std::find(data.P.begin(), data.P.end(), i) != data.P.end()) {
            std::string ride_name = "viol_ridetime_" + std::to_string(i);
            viol_ridetime[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, ride_name.c_str());
        }
    }

    // --- 2. Objective Function ---

    IloExpr objExpr(env);
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        double cost = data.getCost(i, j, k);
        objExpr += cost * x[{i, j, k}];
    }
    for (int k : data.K) 
        for (int i : V_nodes)
            objExpr += alpha  * viol_load[{i, k}];  // alpha * viol_load(i, k)
    for (int k : data.K)
        objExpr += beta * viol_duration[k];         // beta  * viol_duration(k)
    for (int i : V_nodes)
        objExpr += gamma * viol_tw[i];              // gamma * viol_tw(i)
    for (int i : data.P)
        objExpr += tau * viol_ridetime[i];          // tau   * viol_ridetime(i)
    
    model.add(IloMinimize(env, objExpr));
    objExpr.end();

    // --- 3. Constraints ---

    // c1: Each request served once
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

    // c2: Flow conservation
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

    // c3: Flow at Depots
    for (int k : data.K) {
        // Start Depot Rule
        int sk = data.getVehicleStartNode(k);
        IloExpr startExpr(env);
        for (const auto& [ii, jj, kk] : A_k) {
            if (ii == sk && kk == k) {
                startExpr += x[{ii, jj, kk}];
            }
        }
        model.add(startExpr == 1);
        startExpr.end();

        // End Depot Rule
        int ek = data.getVehicleEndNode(k);
        IloExpr endExpr(env);
        for (const auto& [ii, jj, kk] : A_k) {
            if (jj == ek && kk == k) {
                endExpr += x[{ii, jj, kk}];
            }
        }
        model.add(endExpr == 1);
        endExpr.end();
    }

    // c4: Pairing 
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

        double serv = data.getServiceTime(i);
        double trav = data.getTravelTime(i, j);
        double l_i = data.getTimeWindowEnd(i);
        double e_j = data.getTimeWindowStart(j);

        double MAX_TIME_VIOLATION = 1000.0; // Arbitrary large number to allow violations
        double M_ij = std::max(0.0, l_i + MAX_TIME_VIOLATION + serv +  trav - e_j);
        
        model.add(
            u[j] >= u[i] + serv + trav - M_ij * (1 - x[arc])
        );
    }

    // c6: Time Windows (Relaxed)
    for (int i : V_nodes) {
        // Check limits. If vector indices match node IDs
        double ei = data.getTimeWindowStart(i);
        double li = data.getTimeWindowEnd(i);
        
        model.add(u[i] >= ei);
        model.add(u[i] <= li + viol_tw[i]);
    }

    // c7: Max Route Duration
    for (int k : data.K) {
        int sk = data.getVehicleStartNode(k);
        int ek = data.getVehicleEndNode(k);
        double Tmax = data.getVehicleMaxRouteTime(k);
        
        model.add(u[ek] - u[sk] <= Tmax + viol_duration[k]);
    }

    // c8: Precedence
    for (int i : data.P) {
        int delivery_node = i + data.N_requests;
        double serv = data.getServiceTime(i);
        double trav = data.getTravelTime(i, delivery_node);
        
        model.add(
            u[delivery_node] >= u[i] + serv + trav
        );
    }

    // c9: Ride Time Limit
    // u[del, k] - (u[pick, k] + serv) <= L + M(1 - k_serves_i)
    for (int i : data.P) {
        int delivery_node = i + data.N_requests;
        double serv = data.getServiceTime(i);
        double L = data.getMaxRideTime();

        model.add(
            u[delivery_node] - (u[i] + serv) <= L + viol_ridetime[i]
        );
    }
    
    // c10: Load Consistency
    // w[j,k] >= w[i,k] + q[j] - M(1 - x)
    for (const auto& [i, j, k] : A_k) {
        double q_j = data.getDemand(j);
        double Q_k = data.getVehicleCapacity(k);

        double MAX_LOAD_VIOLATION = 1000.0; // Arbitrary large number to allow violations

        double M_ijk = Q_k + MAX_LOAD_VIOLATION;
        
        model.add(
            w[{j, k}] >= w[{i, k}] + q_j - M_ijk * (1 - x[{i, j, k}])
        );
    }

    // c11: Load Feasible Bounds
    for (int i : V_nodes) {
        double qi = data.getDemand(i);
        for (int k : data.K) {
            double Qk = data.getVehicleCapacity(k);
            double lb = std::max(0.0, qi);
            
            model.add(w[{i, k}] >= lb);
            model.add(w[{i, k}] <= Qk + viol_load[{i, k}]);
        }
    }

    // c12: Depot Initial/Final Load
    for (int k : data.K) {
        int sk = data.getVehicleStartNode(k);
        int ek = data.getVehicleEndNode(k);
        model.add(w[{sk, k}] == 0);
        model.add(w[{ek, k}] == 0);
    }
}

bool CPLEXSoftSolver::varExists(int i, int j, int k) const {
    return x.find({i, j, k}) != x.end();
}
