#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <set>

#include "DARPMD_ResultInstance.h"

void DARPMD_ResultInstance::addRoute(int vehicleId, const VehicleRoute& route) {
    routes[vehicleId] = route;
}

void DARPMD_ResultInstance::calculateViolations() {
    SolutionViolations v; // Create an instance to accumulate violations

    double epsilon = 1e-6; // Small tolerance for floating-point comparisons

    for (const auto& [vehicleId, route] : routes) {
        if (route.isEmpty()) continue;

        double routeTravelCost = 0.0;
        std::map<int, double> pickupTimes; // To calculate ride time violations

        for (size_t i = 0; i < route.steps.size(); ++i) {
            const auto& step = route.steps[i];

            // 1. Time Window Violations
            double late = problemInstance.getTimeWindowEnd(step.nodeId);
            double tw_violation = 0.0;
            if (step.arrivalTime > late) tw_violation += (step.arrivalTime - late);
            
            if (tw_violation > epsilon) {
                v.timeWindowsViolation[step.nodeId] = tw_violation;
                v.sumTimeWindows += tw_violation;
            }
            

            // 2. Capacity Violations
            double maxCapacity = problemInstance.getVehicleCapacity(vehicleId);
            if (step.loadAfter > maxCapacity + epsilon) {
                double cap_violation = step.loadAfter - maxCapacity;
                v.capacitiesViolation[step.nodeId] = cap_violation;
                v.sumCapacities += cap_violation;
            }

            // 3. Ride Time Violations
            if (step.type == "Pickup") {
                // We save the time of pickup to calculate ride time when we reach the delivery
                pickupTimes[step.nodeId] = step.arrivalTime; 
            } else if (step.type == "Delivery") {
                int requestId = step.nodeId - problemInstance.N_requests; 
                
                // RideTime = Arrival at Delivery - (Pickup Time + Service Time at Pickup)
                double serviceTimePickup = problemInstance.getServiceTime(requestId);
                double rideTime = step.arrivalTime - (pickupTimes[requestId] + serviceTimePickup);

                double maxRideTime = problemInstance.getMaxRideTime();
                if (rideTime > maxRideTime + epsilon) {
                    double rt_violation = rideTime - maxRideTime;
                    v.rideTimeViolations[requestId] = rt_violation;
                    v.sumRideTime += rt_violation;
                }
            }
        }

        // 4. Route Duration Violations
        double maxRouteDuration = problemInstance.getVehicleMaxRouteTime(vehicleId);
        if (route.routeDuration > maxRouteDuration + epsilon) {
            double dur_violation = route.routeDuration - maxRouteDuration;
            v.routeDurationViolations[vehicleId] = dur_violation;
            v.sumRouteDuration += dur_violation;
        }
        
        v.totalTravelCost += routeTravelCost;
    }

    // 5. Unassigned Requests
    std::set<int> assignedRequests;
    for (const auto& [vehicleId, route] : routes) {
        for (const auto& step : route.steps) {
            if (step.type == "Pickup") {
                assignedRequests.insert(step.nodeId);
            }
        }
    }
    for (int req : problemInstance.P) {
        if (assignedRequests.find(req) == assignedRequests.end()) {
            v.unassignedRequests.push_back(req);
        }
    }

    // 6. Calculate route cost without penalities
    for (const auto& [vehicleId, route] : routes) {
        for (size_t i = 0; i < route.steps.size() - 1; ++i) {
            int from = route.steps[i].nodeId;
            int to = route.steps[i + 1].nodeId;
            v.totalTravelCost += problemInstance.getTravelTime(from, to);
        }
    }

    v.hasViolations = (v.sumTimeWindows > epsilon   || v.sumRideTime > epsilon      ||
                       v.sumCapacities > epsilon    || v.sumRouteDuration > epsilon ||
                       !v.unassignedRequests.empty());

    // Asignar los cálculos finales al std::optional
    this->violations = v;
}

std::string DARPMD_ResultInstance::generateSummaryString() const {
    std::stringstream ss;

    auto vectorPrinter = [](const std::vector<int>& vec) {
        std::stringstream ss;
        ss << "[";
        for (int v : vec) ss << v << " ";
        ss << "]";
        return ss.str();
    };

    ss << "\n=== DARPMD SOLUTION SUMMARY ===\n";
    ss << "Status: " << solverStatus << "\n";
    ss << "Objective Value: " << objectiveValue << "\n";
    ss << "Compute Time: " << solveTime << " s\n";
    ss << "MIP Gap: " << mipGap << "\n";
    ss << "--------------------------------\n\n";

    if (violations.has_value()) {
        if (violations->hasViolations) {
            ss << "\n=== VIOLATIONS ===\n";
            ss << std::left << std::setw(25) << "Total Travel Cost:" << violations->totalTravelCost << "\n";
            ss << std::left << std::setw(25) << "Time Windows:" << std::fixed << std::setprecision(4) << violations->sumTimeWindows << "\n";
            ss << std::left << std::setw(25) << "Ride Times:" << violations->sumRideTime << "\n";
            ss << std::left << std::setw(25) << "Capacities:" << violations->sumCapacities << "\n";
            ss << std::left << std::setw(25) << "Route Durations:" << violations->sumRouteDuration << "\n";
            ss << std::left << std::setw(25) << "Unassigned Reqs:" << vectorPrinter(violations->unassignedRequests) << "\n";
            ss << "--------------------------------\n\n";
        } else {
            ss << "No violations detected. This solution is feasible with respect to all constraints.\n\n";
        }  
    } else {
        ss << "\n[WARNING] Violations have not been calculated yet. Call calculateViolations() to populate this section.\n";
        ss << "--------------------------------\n\n";
    }
    
    auto nodeName = [](int i, int numReq, int numVeh) -> std::string {
        if (i <= numReq) return std::to_string(i)+"+";
        if (i <= 2*numReq) return std::to_string(i - numReq)+"-";
        if (i <= 2*numReq + numVeh) return "V" + std::to_string(i - 2*numReq) + "St";
        return "V" + std::to_string(i - 2*numReq - numVeh) + "En";
    };

    for (const auto& [k, route] : routes) {
        // Header for each vehicle
        ss << "Vehicle " << k << ": ";
        
        if (route.isEmpty()) {
            ss << "UNUSED (Stays at Depot)\n\n";
            continue;
        }

        ss << "Route (Steps: " << route.steps.size() << ")\n";
        
        // Column headers
        ss << "  " // Slight indentation
           << std::left << std::setw(10) << "NodeID"
           << std::setw(15) << "NodeAlias"
           << std::setw(15) << "Type" 
           << std::setw(12) << "ArrTime" 
           << std::setw(12) << "Load" << "\n";

        ss << "  " << std::string(64, '-') << "\n"; // Visual separator

        // Data rows
        for (const auto& step : route.steps) {
            ss << "  " // Slight indentation
               << std::left << std::setw(10) << step.nodeId
               << std::setw(15) << nodeName(step.nodeId, problemInstance.N_requests, problemInstance.K_vehicles)
               << std::setw(15) << step.type 
               << std::setw(12) << std::fixed << std::setprecision(2) << step.arrivalTime 
               << std::setw(12) << std::setprecision(1) << step.loadAfter << "\n";
        }
        ss << "\n";
    }
    ss << "===============================\n";
    
    return ss.str(); // Devolver el string completo
}

void DARPMD_ResultInstance::displaySummary() {
    calculateViolations();
    std::cout << generateSummaryString() << std::endl;
}

void DARPMD_ResultInstance::saveToTxt(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return;
    }
    file << generateSummaryString();
    std::cout << "Summary saved to: " << filename << std::endl;
}

void DARPMD_ResultInstance::saveToJSON(const std::string& filename) const {
    // 1. Create the JSON root object
    json root;

    // 2. Fill Summary
    root["summary"] = {
        {"num_vehicles", problemInstance.K_vehicles},
        {"num_requests", problemInstance.N_requests},
        {"status", solverStatus},
        {"objective", objectiveValue},
        {"time_sec", solveTime},
        {"mip_gap", mipGap}
    };

    // 3. Fill Routes (Automatic array construction)
    root["routes"] = json::array();
    for (const auto& [k, route] : routes) {
        json routeJson;
        routeJson["vehicle_id"] = k;
        
        // Transform C++ vector to JSON array
        json stepsArr = json::array();
        for (const auto& step : route.steps) {
            stepsArr.push_back({
                {"node", step.nodeId},
                {"type", step.type},
                {"arrival", step.arrivalTime},
                {"load", step.loadAfter}
            });
        }
        routeJson["steps"] = stepsArr;
        root["routes"].push_back(routeJson);
    }

    // 4. Metadata Logic (ONLY IF NOT EMPTY)
    if (!metadata.isEmpty()) {
        root["metadata"] = metadata; 
        // The library automatically handles the previous comma and formatting
    }

    // 5. Save to file
    std::ofstream file(filename);
    if (file.is_open()) {
        file << std::setw(4) << root << std::endl;
        std::cout << "JSON saved to: " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
    }
}