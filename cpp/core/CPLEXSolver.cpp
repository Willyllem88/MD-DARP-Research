#include "CPLEXSolver.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <chrono>

CPLEXSolver::CPLEXSolver(DARPMD_ProblemInstance& instance, std::optional<double> timeLimit, bool verbose) 
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

    // Apply Time-Window tightening
    tightenTimeWindows();

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

        for (int i : nodes_k)
            for (int j : nodes_k)
                if (isArcFeasible(i, j, k))
                    A_k.emplace_back(i, j, k);
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

CPLEXSolver::~CPLEXSolver() {
    env.end();
}

bool CPLEXSolver::varExists(int i, int j, int k) const {
    return x.find({i, j, k}) != x.end();
}

void CPLEXSolver::tightenTimeWindows() {
    double L = data.getMaxRideTime();
    uint N = data.N_requests;

    // T: End of the planning horizon
    double T = 0.0;
    for (int e : data.E) {
        T = std::max(T, data.getTimeWindowEnd(e));
    }

    // 1. Tighten Pickup and Delivery Time-Windows
    for (uint i : data.P) {
        uint del_i = i + N; // Corresponding delivery node
        double e_i = data.getTimeWindowStart(i);
        double l_i = data.getTimeWindowEnd(i);
        double e_del_i = data.getTimeWindowStart(del_i);
        double l_del_i = data.getTimeWindowEnd(del_i);

        double serv_i = data.getServiceTime(i);
        double t_i_ni = data.getTravelTime(i, del_i);

        // Tighten the pickup
        double new_e_i = std::max(e_i, e_del_i - L - serv_i);
        double new_l_i = std::min(l_i, std::min(l_del_i - t_i_ni - serv_i, T));
        data.updateTimeWindow(i, new_e_i, new_l_i);

        // Tighten the delivery
        double new_e_del_i = std::max(e_del_i, e_i + serv_i + t_i_ni);
        double new_l_del_i = std::min(l_del_i, std::min(l_i + serv_i + L, T));
        data.updateTimeWindow(del_i, new_e_del_i, new_l_del_i);
    }

    // 2. Tighten depots time-windows
    for (int k : data.K) {
        int sk = data.getVehicleStartNode(k);
        int ek = data.getVehicleEndNode(k);

        // Original values
        double e_sk = data.getTimeWindowStart(sk);
        double l_sk = data.getTimeWindowEnd(sk);
        double e_ek = data.getTimeWindowStart(ek);
        double l_ek = data.getTimeWindowEnd(ek);

        // 1. Therorical bounds
        double e_star_sk = 0.0;
        double l_star_sk = T;
        for (uint j : data.P) {
            e_star_sk = std::min(e_star_sk, data.getTimeWindowStart(j) - data.getTravelTime(sk, j));
            l_star_sk = std::max(l_star_sk, data.getTimeWindowEnd(j)   - data.getTravelTime(sk, j));
        }

        double e_star_ek = 0.0;
        double l_star_ek = T;
        for (uint i : data.D) {
            e_star_ek = std::min(e_star_ek, data.getTimeWindowStart(i) + data.getServiceTime(i) + data.getTravelTime(i, ek));
            l_star_ek = std::max(l_star_ek, data.getTimeWindowEnd(i)   + data.getServiceTime(i) + data.getTravelTime(i, ek));
        }

        // 2. Bounding / Clamping to ensure feasibility
        double new_e_sk = std::clamp(e_star_sk, e_sk, l_sk);
        double new_l_sk = std::clamp(l_star_sk, new_e_sk, l_sk);
        double new_l_ek = std::clamp(l_star_ek, e_ek, l_ek);
        double new_e_ek = std::clamp(e_star_ek, e_ek, new_l_ek);

        // Update the instance of data with the fully tightened time windows
        data.updateTimeWindow(sk, new_e_sk, new_l_sk);
        data.updateTimeWindow(ek, new_e_ek, new_l_ek);

    }
}

bool CPLEXSolver::isArcFeasible(uint i, uint j, uint k) const {
    uint sk = data.getVehicleStartNode(k);
    uint ek = data.getVehicleEndNode(k);
    uint N = data.N_requests;
    double epsilon = 1e-4;

    // 1. Basic logical and structural rules
    if (i == j) return false;  // No self-loops
    if (i == ek) return false; // Does not leave the end
    if (j == sk) return false; // Does not enter the start
    if (data.isDelivery(i) && i == j + N) return false;    // Do not go from delivery to its pickup
    if (data.isVehicleStart(i) && data.isDelivery(j)) return false;      // Do not go from Start depot to delivery
    if (data.isPickup(i) && data.isVehicleEnd(j)) return false;          // Do not go from pickup to End depot
    
    double e_i = data.getTimeWindowStart(i);
    double d_i = data.getServiceTime(i);
    double t_ij = data.getTravelTime(i, j);

    // 2. Feasible return to the depot
    if (j != ek) {
        double Ej_start = std::max(e_i + d_i + t_ij, data.getTimeWindowStart(j));
        double d_j = data.getServiceTime(j);
        double l_ek = data.getTimeWindowEnd(ek);

        if (data.isPickup(j)) {
            // If j is a pickup, we need to consider the corresponding delivery
            uint del_j = j + N;
            double d_del_j = data.getServiceTime(del_j);
            double t_j_delj = data.getTravelTime(j, del_j);
            double t_delj_ek = data.getTravelTime(del_j, ek);

            double e_delj_start = std::max(Ej_start + d_j + t_j_delj, data.getTimeWindowStart(del_j));
            if (e_delj_start + d_del_j + t_delj_ek > l_ek + epsilon)
                return false;
        }
        else {
            double t_j_ek = data.getTravelTime(j, ek);
            if (Ej_start + d_j + t_j_ek > l_ek + epsilon)
                return false;
        }
    }

    // 3. Cross validation
    
    // Case 3a: Two consecutive pickups (i is pickup, j is pickup)
    if (data.isPickup(i) && data.isPickup(j)) {
        uint del_i = i + N;
        uint del_j = j + N;

        if (!(checkPathFeasibility({i, j, del_i, del_j}, k) ||
              checkPathFeasibility({i, j, del_j, del_i}, k))) {
            return false;
        }
    }
    // Case 3b: Two consecutive deliveries (i is delivery, j is delivery)
    else if (data.isDelivery(i) && data.isDelivery(j)) {
        uint pick_i = i - N;
        uint pick_j = j - N;

        if (!(checkPathFeasibility({pick_i, pick_j, i, j}, k) ||
              checkPathFeasibility({pick_j, pick_i, i, j}, k))) {
            return false;
        }
    }
    // Case 3c: Pickup followed by delivery of another passenger (i is pickup, j is delivery)
    else if (data.isPickup(i) && data.isDelivery(j) && (j != i + N)) {
        uint del_i = i + N;
        uint pick_j = j - N;

        if (!checkPathFeasibility({pick_j, i, j, del_i}, k)) {
            return false;
        }

    }
    // Case 3d: Delivery followed by pickup of another passenger (i is delivery, j is pickup)
    else if (data.isDelivery(i) && data.isPickup(j)) {
        uint pick_i = i - N;
        uint del_j = j + N;

        if (!checkPathFeasibility({pick_i, i, j, del_j}, k)) {
            return false;
        }
    }
    
    return true;
}

bool CPLEXSolver::checkPathFeasibility(const std::vector<uint>& path, uint k) const {
    if (path.size() < 2) {
        return true;
    }

    double epsilon = 1e-4;

    // 1. CAPACITY (String lower bound)
    // We calculate the net load variation in the sequence.
    double current_load = 0.0;
    double max_load_seen = 0.0;
    
    for (uint node : path) {
        current_load += data.getDemand(node);
        if (current_load > max_load_seen + epsilon) {
            max_load_seen = current_load;
        }
    }
            
    if (max_load_seen > data.getVehicleCapacity(k) + epsilon) {
        return false;
    }

    // --- 2. TIME WINDOWS (Earliest possible arrival) ---
    // If we simulate the trip at the fastest possible speed and still arrive late, the arc is infeasible.
    double t = data.getTimeWindowStart(path[0]);
    for (size_t idx = 1; idx < path.size(); ++idx) {
        uint prev_n = path[idx - 1];
        uint curr_n = path[idx];
        
        double arrival = std::max(t, data.getTimeWindowStart(prev_n)) 
                       + data.getServiceTime(prev_n) 
                       + data.getTravelTime(prev_n, curr_n);
        
        if (arrival > data.getTimeWindowEnd(curr_n) + epsilon) {
            return false;
        }
            
        t = arrival;
    }

    // --- 3. RIDE TIME (Lower bound of duration) ---
    // We calculate the minimum absolute time between a Pickup and its Delivery within the route.
    for (size_t i = 0; i < path.size(); ++i) {
        uint p_node = path[i];
        
        if (data.isPickup(p_node)) {
            uint d_node = p_node + data.N_requests;
            
            // Check if the route contains the delivery node after the pickup
            auto it = std::find(path.begin() + i + 1, path.end(), d_node);
            
            if (it != path.end()) {
                size_t d_idx = std::distance(path.begin(), it);
                double min_ride_time = 0.0;
                
                for (size_t j = i; j < d_idx; ++j) {
                    min_ride_time += data.getTravelTime(path[j], path[j + 1]);
                    if (j > i) {
                        min_ride_time += data.getServiceTime(path[j]);
                    }
                }

                if (i + 1 < d_idx) {
                    uint next_n = path[i + 1];
                    double l_p = data.getTimeWindowEnd(p_node);
                    double s_p = data.getServiceTime(p_node);
                    double t_p_next = data.getTravelTime(p_node, next_n);
                    double e_next = data.getTimeWindowStart(next_n);
                    
                    double mandatory_wait = std::max(0.0, e_next - (l_p + s_p + t_p_next));
                    min_ride_time += mandatory_wait;
                }

                if (min_ride_time > data.getMaxRideTime() + epsilon) {
                    return false;
                }
            }
        }
    }

    return true;
}

void CPLEXSolver::buildModel() {
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

    // --- 2. Objective Function ---

    IloExpr objExpr(env);
    for (const auto& arc : A_k) {
        auto [i, j, k] = arc;
        double cost = data.getCost(i, j, k);
        objExpr += cost * x[{i, j, k}];
    }
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

        double M_ij = std::max(0.0, l_i + serv + trav - e_j);
        
        model.add(
            u[j] >= u[i] + serv + trav - M_ij * (1 - x[arc])
        );
    }

    // c6: Time Windows
    for (int i : V_nodes) {
        // Check limits. If vector indices match node IDs
        double ei = data.getTimeWindowStart(i);
        double li = data.getTimeWindowEnd(i);
        
        model.add(u[i] >= ei);
        model.add(u[i] <= li);
    }

    // c7: Max Route Duration
    for (int k : data.K) {
        int sk = data.getVehicleStartNode(k);
        int ek = data.getVehicleEndNode(k);
        double Tmax = data.getVehicleMaxRouteTime(k);
        
        model.add(u[ek] - u[sk] <= Tmax);
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
            u[delivery_node] - (u[i] + serv) <= L
        );
    }

    // c10: Load Consistency
    // w[j,k] >= w[i,k] + q[j] - M(1 - x)
    for (const auto& [i, j, k] : A_k) {
        double q_j = data.getDemand(j);
        double Q_k = data.getVehicleCapacity(k);

        double M_ijk = Q_k;
        
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
            double ub = std::min(Qk, Qk + qi);
            
            model.add(w[{i, k}] >= lb);
            model.add(w[{i, k}] <= ub);
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

void CPLEXSolver::solve() {
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

int CPLEXSolver::getNumberOfConstraints() const {
    return cplex.getNrows();
}

int CPLEXSolver::getNumberOfVariables() const {
    return cplex.getNcols();
}

DARPMD_ResultInstance CPLEXSolver::getResult() const {
    DARPMD_ResultInstance result(data);

    // 1. General Solution Info
    try {
        result.objectiveValue = this->objectiveValue;
        result.solveTime = this->solveTime;
        result.mipGap = this->mipGap;
        result.solverStatus = (cplex.getStatus() == IloAlgorithm::Optimal) ? "Optimal" : "Feasible";
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

    return result;
}

double CPLEXSolver::solveLPRelaxation() {
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