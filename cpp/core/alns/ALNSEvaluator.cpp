#include "ALNSEvaluator.h"

#include "../ALNSSolver.h"

ALNSEvaluator::ALNSEvaluator(const DARPMD_ProblemInstance& data, 
                             const ALNSParams& params)
    : data(data), params(params) { };

void ALNSEvaluator::evaluateRoute(ALNSRoute& route) {
    // Reset metrics
    route.distanceCost = 0.0;
    route.timeWindowViolation = 0.0;
    route.vehicleMaxRouteTimeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;

    if (route.sequence.empty()) return;

    int q = route.sequence.size() - 1; // Last index (end depot)
    if ((int)route.arrivalTimes.size() < data.max_node_id + 1) {
        route.resize(data.max_node_id + 1);
    }

    // Temporary arrays for the evaluation procedure
    std::vector<double>& A = route.A;
    std::vector<double>& W = route.W;
    std::vector<double>& B = route.B;
    std::vector<double>& D = route.D;
    std::vector<double>& Fi = route.Fi;
    std::vector<int>& id2pos = route.id2pos;
    if (route.sequence.size() != A.size()) {
        A.resize(route.sequence.size());
        W.resize(route.sequence.size());
        B.resize(route.sequence.size());
        D.resize(route.sequence.size());
        Fi.resize(route.sequence.size());
    }
    if (route.id2pos.size() != (size_t)data.max_node_id + 1) {
        route.id2pos.resize(data.max_node_id + 1, -1);
    }

    std::vector<double> pickup_D(data.N_requests + 1, -1.0);

    for (int i = 0; i <= q; ++i) {
        int v = route.sequence[i];
        id2pos[v] = i;
    }

    // Helper Lambda: Propagate times forward from a given index
    auto propagateForward = [&](int startIndex) {
        for (int i = startIndex; i <= q; ++i) {
            int u = route.sequence[i - 1];
            int v = route.sequence[i];
            
            A[i] = D[i - 1] + data.getTravelTime(u, v);
            double earlyTW = data.getTimeWindowStart(v);
            
            W[i] = std::max(0.0, earlyTW - A[i]);
            B[i] = A[i] + W[i];
            D[i] = B[i] + data.getServiceTime(v);

            if (data.isPickup(v)) {
                pickup_D[v] = D[i];
            }
        }
    };

    // Helper Lambda: Calculate Forward Time Slack (F_i)
    auto calculateF = [&](int i) {
        double F = std::numeric_limits<double>::infinity();
        double sumW = 0.0;
        
        for (int j = i; j <= q; ++j) {
            if (j > i) {
                sumW += W[j];
            }
            int vj = route.sequence[j];
            double lj = data.getTimeWindowEnd(vj);
            double Pj = 0.0;
            
            // If vj is a delivery node, calculate current ride time (Pj)
            if (data.isDelivery(vj)) {
                int pickupId = vj - data.N_requests; // Assuming D_id = P_id + N_requests
                Pj = B[j] - pickup_D[pickupId]; // Ride time = Begin service at D - Departure at P
            }
            
            double timeWindowMargin = lj - B[j];
            double rideTimeMargin = data.getMaxRideTime() - Pj;
            
            double margin = std::max(0.0, std::min(timeWindowMargin, rideTimeMargin));
            F = std::min(F, sumW + margin);
        }
        return F;
    };

    // Helper: Calculate sum of waiting times between two indices exclusive of start, inclusive of end
    auto getSumW = [&](int start, int end) {
        double sum = 0.0;
        for (int p = start + 1; p < end; ++p) sum += W[p];
        return sum;
    };

    // PHASE 1: Time Window Minimization
    D[0] = std::max(0.0, data.getTimeWindowStart(route.sequence[0]));
    B[0] = D[0]; // Assuming depot service time is 0
    propagateForward(1);

    // PHASE 2: Route Duration Minimization
    double F0 = calculateF(0);
    double sumW_depot = getSumW(0, q);
    
    D[0] = data.getTimeWindowStart(route.sequence[0]) + std::min(F0, sumW_depot);
    propagateForward(1); // Re-calculate A, W, B, D

    // PHASE 3: Ride Time Minimization
    for (int j = 1; j < q; ++j) {
        int vj = route.sequence[j];
        if (data.isPickup(vj)) {
            double Fj = calculateF(j);
            double sumW_j = getSumW(j, q);
            
            double shift = std::min(Fj, sumW_j);
            if (shift > 0.0) {
                B[j] += shift;
                D[j] = B[j] + data.getServiceTime(vj);
                propagateForward(j + 1); // Propagate changes to the rest of the route
            }
        }
    }

    // PHASE 4: Final Evaluation & Constraint Checks
    double currentLoad = 0.0;
    double capacity = data.getVehicleCapacity(route.vehicleId);
    
    route.arrivalTimes[route.sequence[0]] = A[0];
    route.loads[route.sequence[0]] = 0.0;

    for (int i = 1; i <= q; ++i) {
        int u = route.sequence[i - 1];
        int v = route.sequence[i];

        // Assign definitive arrival times
        route.arrivalTimes[v] = A[i];

        // Distance Cost
        route.distanceCost += data.getCost(u, v, route.vehicleId);

        // Capacity Constraint
        currentLoad += data.getDemand(v);
        route.loads[v] = currentLoad;
        if (currentLoad > capacity) {
            route.loadViolation += (currentLoad - capacity);
        }

        // Time Window Constraint
        double lateTW = data.getTimeWindowEnd(v);
        if (B[i] > lateTW) {
            route.timeWindowViolation += (B[i] - lateTW);
        }

        // Ride Time Constraint
        if (data.isDelivery(v)) {
            int pickupId = v - data.N_requests;
            int pickupPos = id2pos[pickupId];
            double rideTime = B[i] - D[pickupPos];
            if (rideTime > data.getMaxRideTime()) {
                route.rideTimeViolation += (rideTime - data.getMaxRideTime());
            }
        }
    }

    // Max Route Duration Constraint
    double duration = A[q] - D[0]; // Arrival at end depot - Departure from start depot
    double maxRouteTime = data.getVehicleMaxRouteTime(route.vehicleId);
    if (duration > maxRouteTime) {
        route.vehicleMaxRouteTimeViolation += (duration - maxRouteTime);
    }

    // PHASE 5: calculate F_i for all nodes (for potential use in move evaluations)
    route.Fi.assign(q + 1, 0.0);
    for (int i = q; i >= 0; --i) {
        int v = route.sequence[i];
        
        double margin_TW = data.getTimeWindowEnd(v) - route.B[i];
        
        double margin_Ride = std::numeric_limits<double>::infinity();
        if (data.isDelivery(v)) {
            int pickupId = v - data.N_requests;
            int pickupPos = id2pos[pickupId];
            double rideTime = route.B[i] - route.D[pickupPos];
            margin_Ride = data.getMaxRideTime() - rideTime;
        }
        
        double local_margin = std::max(0.0, std::min(margin_TW, margin_Ride));
        
        if (i == q) {
            route.Fi[i] = local_margin;
        } else {
            route.Fi[i] = std::min(local_margin, route.W[i + 1] + route.Fi[i + 1]);
        }
    }


    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeWindowViolation
                    + params.vehicleMaxRouteTimePenalty * route.vehicleMaxRouteTimeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeWindowViolation <= 1e-6 && 
                        route.vehicleMaxRouteTimeViolation <= 1e-6 &&
                        route.loadViolation <= 1e-6 && 
                        route.rideTimeViolation <= 1e-6);
}

void ALNSEvaluator::evaluateRouteGreedy(ALNSRoute& route) {
    // Reset metrics
    route.distanceCost = 0.0;
    route.timeWindowViolation = 0.0;
    route.vehicleMaxRouteTimeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;

    if (route.sequence.empty()) return;

    if ((int)route.arrivalTimes.size() < data.max_node_id + 1) {
        route.resize(data.max_node_id + 1);
    }

    // Assumption: pickup ids 1..n
    static thread_local std::vector<double> pickupTimes;
    if ((int)pickupTimes.size() < data.N_requests + 1) pickupTimes.resize(data.N_requests + 1);
    std::fill(pickupTimes.begin(), pickupTimes.end(), -1.0);

    double currentTime = 0.0;
    double currentLoad = 0.0;
    
    // 1. Initialize at Start Depot
    int startNode = route.sequence.front();
    currentTime = std::max(0.0, data.getTimeWindowStart(startNode)); 
    
    route.arrivalTimes[startNode] = currentTime;
    route.loads[startNode] = 0.0;

    for (size_t i = 0; i < route.sequence.size() - 1; ++i) {
        int u = route.sequence[i];
        int v = route.sequence[i+1];

        // --- Distance ---
        route.distanceCost += data.getCost(u, v, route.vehicleId);

        // --- Time ---
        double serviceTime = data.getServiceTime(u);
        double travelTime = data.getTravelTime(u, v);
        
        double arrivalAtV = currentTime + serviceTime + travelTime;
        
        // Time Window Check at V
        double earlyTW = data.getTimeWindowStart(v);
        double lateTW = data.getTimeWindowEnd(v);

        // If early, we wait
        if (arrivalAtV < earlyTW) {
            arrivalAtV = earlyTW;
        }
        
        // If late, penalty
        if (arrivalAtV > lateTW) {
            route.timeWindowViolation += (arrivalAtV - lateTW);
            // In a soft TW context, we assume we arrive 'late' but continue
            // To prevent massive propagation, sometimes we clamp, but here we let it flow
        }

        currentTime = arrivalAtV;
        route.arrivalTimes[v] = currentTime;

        // --- Capacity ---
        double demand = data.getDemand(v);
        currentLoad += demand;
        route.loads[v] = currentLoad;

        double capacity = data.getVehicleCapacity(route.vehicleId);
        if (currentLoad > capacity) {
            route.loadViolation += (currentLoad - capacity);
        }

        // --- Ride Time Logic ---
        // If v is a pickup node (in P)
        if (data.isPickup(v)) {
            pickupTimes[v] = currentTime;
        }
        // If v is a delivery node (in D)
        else if (data.isDelivery(v)) {
            // Find corresponding pickup ID. Assuming D_id = P_id + N_requests
            int pickupId = v - data.N_requests;
            
            if (pickupTimes[pickupId] >= 0) {
                double rideTime = currentTime - (pickupTimes[pickupId] + data.getServiceTime(pickupId));
                if (rideTime > data.getMaxRideTime()) {
                    route.rideTimeViolation += (rideTime - data.getMaxRideTime());
                }
            }
        }
    }

    // Check Max Route Duration
    int endNode = route.sequence.back();
    double duration = route.arrivalTimes[endNode] - route.arrivalTimes[startNode];
    if (duration > data.getVehicleMaxRouteTime(route.vehicleId)) {
        route.vehicleMaxRouteTimeViolation += (duration - data.getVehicleMaxRouteTime(route.vehicleId));
    }

    // Final Cost Aggregation
    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeWindowViolation
                    + params.vehicleMaxRouteTimePenalty * route.vehicleMaxRouteTimeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeWindowViolation <= 1e-6 && 
                        route.vehicleMaxRouteTimeViolation <= 1e-6 &&
                        route.loadViolation <= 1e-6 && 
                        route.rideTimeViolation <= 1e-6);
}

void ALNSEvaluator::evaluateSolutionGreedy(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    sol.hasViolations = false;
    for (auto& r : sol.routes) {
        evaluateRouteGreedy(r);
        sol.objectiveValue += r.totalCost;
        if (!r.isFeasible) sol.hasViolations = true;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

void ALNSEvaluator::evaluateSolution(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    sol.hasViolations = false;
    for (auto& r : sol.routes) {
        evaluateRoute(r);
        sol.objectiveValue += r.totalCost;
        if (!r.isFeasible) sol.hasViolations = true;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

double ALNSEvaluator::calculateExactDelta(const ALNSRoute& route, int requestId, int i, int j) {
    // Simulate a full evaluation of the route with the new request inserted at positions i (pickup) and j (delivery)
    ALNSRoute temp = route;
    temp.sequence.insert(temp.sequence.begin() + i, requestId); 
    temp.sequence.insert(temp.sequence.begin() + j + 1, requestId + data.N_requests);
    evaluateRoute(temp);

    double delta = temp.totalCost - route.totalCost;

    return delta;
}

double ALNSEvaluator::calculateGreedyDelta(const ALNSRoute& route, int requestId, int i, int j, double upper_bound) {
    int P_node = requestId; 
    int D_node = requestId + data.N_requests; 
    
    double capacity = data.getVehicleCapacity(route.vehicleId);
    int numNodes = route.sequence.size();
    
    // Accumulators of the evaluated segment
    double old_distance = 0.0, old_tw_viol = 0.0, old_load_viol = 0.0, old_ride_viol = 0.0;
    double new_distance = 0.0, new_tw_viol = 0.0, new_load_viol = 0.0, new_ride_viol = 0.0;
    
    double prev_D_new = route.D[i - 1];
    double current_load_new = route.loads[route.sequence[i - 1]];
    int prev_node_new = route.sequence[i - 1];
    
    double D_P = -1.0; 
    std::vector<double> new_D_times(numNodes, 0.0);
    double final_arrival_time_new = 0.0;
    double partial_delta = 0.0;

    // Lambda to simulate a node and accumulate metrics, with early stopping if we exceed upper_bound
    auto simulateNode = [&](int curr_node, int original_idx, bool is_inserted) {
        new_distance += data.getCost(prev_node_new, curr_node, route.vehicleId);
        
        double A_curr = prev_D_new + data.getTravelTime(prev_node_new, curr_node);
        double W_curr = std::max(0.0, data.getTimeWindowStart(curr_node) - A_curr);
        
        double B_curr = A_curr + W_curr;
        prev_D_new = B_curr + data.getServiceTime(curr_node);
        final_arrival_time_new = A_curr; 
        
        double lateTW = data.getTimeWindowEnd(curr_node);
        if (B_curr > lateTW) new_tw_viol += (B_curr - lateTW);
        
        current_load_new += data.getDemand(curr_node);
        if (current_load_new > capacity) new_load_viol += (current_load_new - capacity);
        
        if (data.isDelivery(curr_node)) {
            double p_departure;
            if (curr_node == D_node) {
                p_departure = D_P; 
            } else {
                int p_id = curr_node - data.N_requests;
                int p_pos = route.id2pos[p_id];
                if (p_pos < 0) {
                    p_departure = 0.0; // Fallback
                } else if (p_pos < i) {
                    p_departure = route.D[p_pos]; 
                } else if (p_pos <= original_idx) {
                    p_departure = new_D_times[p_pos]; 
                } else {
                    p_departure = route.D[p_pos];
                }
            }
            double rt = B_curr - p_departure;
            if (rt > data.getMaxRideTime()) new_ride_viol += (rt - data.getMaxRideTime());
        }
        
        if (curr_node == P_node) {
            D_P = prev_D_new;
        } else if (!is_inserted && original_idx != -1) {
            new_D_times[original_idx] = prev_D_new;
        }
        prev_node_new = curr_node;
    };

    bool broke_early = false;

    // Unified loop: we advance node by node comparing old and new realities
    for (int k = i; k < numNodes; ++k) {
        int curr_old = route.sequence[k];
        int prev_old = route.sequence[k - 1];
        
        // 1. Accumulate old metrics for this step k
        old_distance += data.getCost(prev_old, curr_old, route.vehicleId);
        
        double l = route.loads[curr_old];
        if (l > capacity) old_load_viol += (l - capacity);
        
        double lateTW = data.getTimeWindowEnd(curr_old);
        if (route.B[k] > lateTW) old_tw_viol += (route.B[k] - lateTW);
        
        if (data.isDelivery(curr_old)) {
            int p_id = curr_old - data.N_requests;
            int p_pos = route.id2pos[p_id];
            double rt = route.B[k] - route.D[p_pos];
            if (rt > data.getMaxRideTime()) old_ride_viol += (rt - data.getMaxRideTime());
        }
        
        // 2. Simulate the new metrics for this step k
        if (k == i && k == j) {
            simulateNode(P_node, -1, true);
            simulateNode(D_node, -1, true);
        } else if (k == i) {
            simulateNode(P_node, -1, true);
        } else if (k == j) {
            simulateNode(D_node, -1, true);
        }
        simulateNode(curr_old, k, false);
        
        // 3. Calculate partial delta
        double delta_dist = new_distance - old_distance;
        double delta_tw   = new_tw_viol - old_tw_viol;
        double delta_load = new_load_viol - old_load_viol;
        double delta_ride = new_ride_viol - old_ride_viol;
        
        partial_delta = delta_dist 
                      + params.timeWindowPenalty * delta_tw
                      + params.capacityPenalty * delta_load
                      + params.rideTimePenalty * delta_ride;
                      
        // Early Stopping:
        // If at any point the partial delta exceeds the upper bound, we can stop the simulation 
        // and return infinity (or a very large number) to indicate this move is not promising.
        if (partial_delta > upper_bound) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Propagation stop of time (Δt ≈ 0): 
        // If we have already inserted the delivery and the departure time at this node matches 
        // the old one, we can stop propagating further as the rest of the route will be unaffected
        // in terms of timing.
        if (k >= j) {
            // We use 1e-4 to prevent precision issues with floating-point arithmetic
            if (std::abs(prev_D_new - route.D[k]) < 1e-4) {
                broke_early = true;
                break;
            }
        }
    }
    
    // 4. Calculate Delta of Route Duration
    double old_duration = route.A.back() - route.D[0];
    double old_dur_viol = std::max(0.0, old_duration - data.getVehicleMaxRouteTime(route.vehicleId));
    double new_dur_viol = old_dur_viol; 
    
    // If we did not break the loop early, recalculate the total duration
    if (!broke_early) {
        double new_duration = final_arrival_time_new - route.D[0];
        new_dur_viol = std::max(0.0, new_duration - data.getVehicleMaxRouteTime(route.vehicleId));
    }
    
    double delta_dur = new_dur_viol - old_dur_viol;
    partial_delta += params.vehicleMaxRouteTimePenalty * delta_dur;
    
    return partial_delta;
}
