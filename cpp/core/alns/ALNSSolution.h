#pragma once

#include "ALNSRoute.h"

#include <vector> 
#include <set>
#include <iostream>

// Internal representation of a full solution
struct ALNSSolution {
    std::vector<ALNSRoute> routes; // One per vehicle
    std::set<int> unassignedRequests; // IDs of requests in P not served
    double objectiveValue = 0.0; // Total penalized cost

    void print() {
        std::cout << "  Routes:" << std::endl;
        for (const auto& r : routes) {
            std::cout << "    Vehicle " << r.vehicleId << ": ";
            for (int node : r.sequence) {
                std::cout << node << " ";
            }
            std::cout << "| Cost: " << r.totalCost 
                    << " | TW Violation: " << r.timeWindowViolation 
                    << " | Load Violation: " << r.loadViolation 
                    << " | Ride Time Violation: " << r.rideTimeViolation
                    << std::endl;
        }
        std::cout << "  Unassigned Requests: ";
        for (int req : unassignedRequests) {
            std::cout << req << " ";
        }
        std::cout << std::endl;
    }
};