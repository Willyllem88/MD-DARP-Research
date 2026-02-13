#include "ALNSEvaluator.h"

#include "../ALNSSolver.h"

ALNSEvaluator::ALNSEvaluator(ALNSSolver& solver, 
                             const DARPMD_ProblemInstance& data, 
                             const ALNSParams& params)
    : data(data), params(params), solver(solver) { };

void ALNSEvaluator::evaluateRoute(ALNSRoute& route) {
    route.distanceCost = 0.0;
    //TODO: timeWindowViolation and vehicleMaxRouteTimeViolation should be calculated separately
    route.timeWindowViolation = 0.0;
    route.vehicleMaxRouteTimeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;
    route.arrivalTimes.clear();
    route.loads.clear();

    if (route.sequence.empty()) return;

    double currentTime = 0.0;
    double currentLoad = 0.0;
    
    // 1. Initialize at Start Depot
    int startNode = route.sequence.front();
    double startWindow = data.getTimeWindowStart(startNode);
    currentTime = std::max(0.0, startWindow); 
    
    route.arrivalTimes[startNode] = currentTime;
    route.loads[startNode] = 0.0;

    std::map<int, double> pickupTimes;

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
            if (pickupTimes.count(pickupId)) {
                double rideTime = currentTime - (pickupTimes[pickupId] + data.getServiceTime(pickupId));
                if (rideTime > data.max_ride_time) {
                    route.rideTimeViolation += (rideTime - data.max_ride_time);
                }
            }
        }
    }

    // Check Max Route Duration
    int endNode = route.sequence.back();
    double duration = route.arrivalTimes[endNode] - route.arrivalTimes[startNode];
    if (duration > data.max_route_time.at(route.vehicleId)) {
        route.vehicleMaxRouteTimeViolation += (duration - data.max_route_time.at(route.vehicleId));
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

void ALNSEvaluator::evaluateSolution(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRoute(r);
        sol.objectiveValue += r.totalCost;
        
        // Add valid routes to pool for later Set Partitioning (TODO: maybe only those that are feasible or below a certain cost threshold)
        if (r.isFeasible) {
            solver.addRouteToPool(r);
        }
    }
    // Penalize unassigned
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

bool ALNSEvaluator::solutionHasViolations(const ALNSSolution& sol) const {
    for (const auto& r : sol.routes) {
        if (!r.isFeasible) return true;
    }
    return false;
}