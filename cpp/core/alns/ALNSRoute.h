#pragma once

#include <vector> 
#include <map>

// Internal representation of a single vehicle route
struct ALNSRoute {
    int vehicleId;
    std::vector<int> sequence; // Sequence of Node IDs (Depot -> ... -> Depot)
    
    // Evaluation metrics
    double distanceCost = 0.0;
    double timeWindowViolation = 0.0;
    double vehicleMaxRouteTimeViolation = 0.0;
    double loadViolation = 0.0;
    double rideTimeViolation = 0.0;
    double totalCost = 0.0; // penalized cost
    
    bool isFeasible = false;

    // Timestamps and loads for reconstruction
    std::map<int, double> arrivalTimes;
    std::map<int, double> loads;
};

struct RouteSequenceHash {
    std::size_t operator()(const std::vector<int>& seq) const {
        std::size_t hash = 0;
        for (int nodeId : seq) {
            hash ^= std::hash<int>{}(nodeId) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};