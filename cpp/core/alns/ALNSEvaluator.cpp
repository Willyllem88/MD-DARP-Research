#include "ALNSEvaluator.h"

#include "../ALNSSolver.h"

ALNSEvaluator::ALNSEvaluator(const MDDARP_ProblemInstance& data, 
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
    int r = route.sequence.size() - 1; // Last index (end depot)

    route.initializeNodeArrays(data.max_node_id);
    for (int i = 0; i <= r; ++i) {
        int v = route.sequence[i];
        route.id2pos[v] = i;
        if (i > 0)
            route.setLoadByPos(i, data.getDemand(v) + route.getLoadByPos(i - 1));
        else
            route.setLoadByPos(i, 0.0); // Start depot load is 0
    }

    // Helper Lambda: Propagate times forward from a given index
    auto propagateForward = [&](int startIndex) {
        for (int i = startIndex; i <= r; ++i) {
            int u = route.sequence[i - 1];
            int v = route.sequence[i];
            
            route.setAByPos(i, route.getDByPos(i - 1) + data.getTravelTime(u, v));
            double earlyTW = data.getTimeWindowStart(v);
            
            route.setWByPos(i, std::max(0.0, earlyTW - route.getAByPos(i)));
            route.setBByPos(i, route.getAByPos(i) + route.getWByPos(i));
            route.setDByPos(i, route.getBByPos(i) + data.getServiceTime(v));
        }
    };

    // Helper Lambda: Calculate Forward Time Slack (F_i)
    auto calculateF = [&](int i) {
        double F = std::numeric_limits<double>::infinity();
        double sumW = 0.0;
        
        for (int j = i; j <= r; ++j) {
            if (j > i) {
                sumW += route.getWByPos(j);
            }
            int vj = route.sequence[j];
            double lj = data.getTimeWindowEnd(vj);
            double Pj = 0.0;

            double timeWindowMargin = lj - route.getBByPos(j);
            double rideTimeMargin = std::numeric_limits<double>::infinity();

            
            // If vj is a delivery node, calculate current ride time (Pj)
            if (data.isDelivery(vj)) {
                int pickupId = vj - data.N_requests; // Assuming D_id = P_id + N_requests
                int pickupPos = route.id2pos[pickupId];

                if (pickupPos < i) {
                    Pj = route.getBByPos(j) - route.getD(pickupId);
                    rideTimeMargin = data.getMaxRideTime() - Pj;
                }
            }
            
            double margin = std::max(0.0, std::min(timeWindowMargin, rideTimeMargin));
            F = std::min(F, sumW + margin);
        }
        return F;
    };

    // Helper: Calculate sum of waiting times between two indices exclusive of start, inclusive of end
    auto getSumW = [&](int start, int end) {
        double sum = 0.0;
        for (int p = start + 1; p < end; ++p) sum += route.getWByPos(p);
        return sum;
    };

    // PHASE 1: Time Window Minimization
    route.setDByPos(0, std::max(0.0, data.getTimeWindowStart(route.sequence[0])));
    route.setBByPos(0, route.getDByPos(0));
    propagateForward(1);

    // PHASE 2: Route Duration Minimization
    double F0 = calculateF(0);
    double sumW_depot = getSumW(0, r);
    
    route.setDByPos(0, data.getTimeWindowStart(route.sequence[0]) + std::min(F0, sumW_depot));
    propagateForward(1); // Re-calculate A, W, B, D

    // PHASE 3: Ride Time Minimization
    for (int j = 1; j < r; ++j) {
        int vj = route.sequence[j];
        if (data.isPickup(vj)) {
            double Fj = calculateF(j);
            double sumW_j = getSumW(j, r);
            
            double shift = std::min(Fj, sumW_j);
            if (shift > 0.0) {
                route.setBByPos(j, route.getBByPos(j) + shift);
                route.setDByPos(j, route.getBByPos(j) + data.getServiceTime(vj));
                propagateForward(j + 1); // Propagate changes to the rest of the route
            }
        }
    }

    // PHASE 4: Final Evaluation & Constraint Checks
    double capacity = data.getVehicleCapacity(route.vehicleId);
    
    for (int i = 1; i <= r; ++i) {
        int u = route.sequence[i - 1];
        int v = route.sequence[i];

        // Distance Cost
        route.distanceCost += data.getCost(u, v, route.vehicleId);

        // Capacity Constraint
        if (route.getLoadByPos(i) > capacity) {
            route.loadViolation += (route.getLoadByPos(i) - capacity);
        }

        // Time Window Constraint
        double lateTW = data.getTimeWindowEnd(v);
        if (route.getBByPos(i) > lateTW) {
            route.timeWindowViolation += (route.getBByPos(i) - lateTW);
        }

        // Ride Time Constraint
        if (data.isDelivery(v)) {
            int pickupId = v - data.N_requests;
            int pickupPos = route.id2pos[pickupId];
            double rideTime = route.getBByPos(i) - route.getDByPos(pickupPos);
            if (rideTime > data.getMaxRideTime()) {
                route.rideTimeViolation += (rideTime - data.getMaxRideTime());
            }
        }
    }

    // Max Route Duration Constraint
    double duration = route.getAByPos(r) - route.getDByPos(0); // Arrival at end depot - Departure from start depot
    double maxRouteTime = data.getVehicleMaxRouteTime(route.vehicleId);
    if (duration > maxRouteTime) {
        route.vehicleMaxRouteTimeViolation += (duration - maxRouteTime);
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

    sol.initNodeDirectory(data.max_node_id); // TODO: maybe update it in the ALNSRoute evaluation instead of here, to avoid double work
    sol.updateNodeToRouteMapping();
}

double ALNSEvaluator::calculateDelta(const ALNSRoute& route, ALNSRoute& temp, int requestId, int i, int j) {
    // Simulate a full evaluation of the route with the new request inserted at positions i (pickup) and j (delivery)
    temp.sequence.insert(temp.sequence.begin() + i, requestId); 
    temp.sequence.insert(temp.sequence.begin() + j + 1, requestId + data.N_requests);
    evaluateRoute(temp);
    temp.sequence.erase(temp.sequence.begin() + j + 1); // Revert to original state
    temp.sequence.erase(temp.sequence.begin() + i);

    double delta = temp.totalCost - route.totalCost;

    return delta;
}
