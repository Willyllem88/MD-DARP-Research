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
    std::vector<double> A(q + 1, 0.0); // Arrival times
    std::vector<double> W(q + 1, 0.0); // Waiting times
    std::vector<double> B(q + 1, 0.0); // Beginning of service times
    std::vector<double> D(q + 1, 0.0); // Departure times

    std::vector<double> pickup_D(data.N_requests + 1, -1.0);

    // Helper Lambda: Propagate times forward from a given index
    auto propagateForward = [&](int startIndex) {
        for (int i = startIndex; i <= q; ++i) {
            int u = route.sequence[i - 1];
            int v = route.sequence[i];

            if (data.isPickup(v)) {
                pickup_D[v] = D[i - 1]; // Store departure time of pickup for later ride time calculation
            }
            
            A[i] = D[i - 1] + data.getTravelTime(u, v);
            double earlyTW = data.getTimeWindowStart(v);
            
            W[i] = std::max(0.0, earlyTW - A[i]);
            B[i] = A[i] + W[i];
            D[i] = B[i] + data.getServiceTime(v);
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
            // TODO: This could be mapped directly
            for (int k = 0; k < i; ++k) {
                if (route.sequence[k] == pickupId) {
                    double rideTime = B[i] - D[k];
                    if (rideTime > data.getMaxRideTime()) {
                        route.rideTimeViolation += (rideTime - data.getMaxRideTime());
                    }
                    break;
                }
            }
        }
    }

    // Max Route Duration Constraint
    double duration = A[q] - D[0]; // Arrival at end depot - Departure from start depot
    double maxRouteTime = data.getVehicleMaxRouteTime(route.vehicleId);
    if (duration > maxRouteTime) {
        route.vehicleMaxRouteTimeViolation += (duration - maxRouteTime);
    }

    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeWindowViolation
                    + params.vehicleMaxRouteTimePenalty * route.vehicleMaxRouteTimeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeWindowViolation == 0 && 
                        route.vehicleMaxRouteTimeViolation == 0 &&
                        route.loadViolation == 0 && 
                        route.rideTimeViolation == 0);
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

        // If early, we wait (Time warp is not a violation usually, but waiting)
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

    route.isFeasible = (route.timeWindowViolation == 0 && 
                        route.vehicleMaxRouteTimeViolation == 0 &&
                        route.loadViolation == 0 && 
                        route.rideTimeViolation == 0);
}

void ALNSEvaluator::evaluateSolutionGreedy(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRouteGreedy(r);
        sol.objectiveValue += r.totalCost;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

void ALNSEvaluator::evaluateSolution(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRoute(r);
        sol.objectiveValue += r.totalCost;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

bool ALNSEvaluator::solutionHasViolations(const ALNSSolution& sol) const {
    for (const auto& r : sol.routes) {
        if (!r.isFeasible) return true;
    }
    return false;
}