#pragma once

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <optional>

#include "DARPMD_ProblemInstance.h"
#include "Metadata.h"

#include "../includes/json.hpp"
using json = nlohmann::json;

// Structure for representing the visit to a specific node
struct RouteStep {
    int nodeId;             // ID of the node
    std::string type;       // "DepotStart", "DepotEnd", "Pickup", "Delivery"
    double beginServiceTime;     // Value of the variable u
    double loadAfter;       // Load of the vehicle after visiting the node (variable w)
};

// Structure for representing the complete route of a vehicle
struct VehicleRoute {
    int vehicleId;                // ID of the vehicle (1, 2, ..., K)
    double routeCost;             // Specific cost of this route (if calculated)
    double routeDuration;         // Total time used
    std::vector<RouteStep> steps; // Ordered sequence of visits

    bool isEmpty() const { return steps.size() <= 2; } // If it only has Start and End
};

// Main class to store the global result
class DARPMD_ResultInstance {
public:
    // --- Solution Metadata ---
    double objectiveValue;
    std::string solverStatus; // "Optimal", "Feasible", "Semi-Feasible", "Infeasible"
    double solveTime;         // Computation time
    double mipGap;            // Final MIP gap
    // --- Route Data ---
    // Map: Vehicle ID -> Route Object
    std::map<int, VehicleRoute> routes;

    Metadata metadata; // Additional information (city, coordinates, etc.)
    DARPMD_ProblemInstance problemInstance;

    struct SolutionViolations {
        double totalTravelCost = 0.0;
        double totalCost = 0.0;
        
        std::map<int, double> timeWindowsViolation;     // nodeId -> violation amount
        double sumTimeWindows = 0.0;
        std::map<int, double> rideTimeViolations;       // requestId -> violation amount
        double sumRideTime = 0.0;
        std::map<int, double> capacitiesViolation;      // vehicleId -> violation amount
        double sumCapacities = 0.0;
        std::map<int, double> routeDurationViolations;  // vehicleId -> violation amount
        double sumRouteDuration = 0.0;
        
        std::vector<int> unassignedRequests;

        bool hasViolations = false;
    };

    std::optional<SolutionViolations> violations;

    // --- Constructor ---
    DARPMD_ResultInstance() = delete;
    DARPMD_ResultInstance(const DARPMD_ProblemInstance& problem):
        objectiveValue(0.0),
        solverStatus("NotSolved"),
        solveTime(0.0),
        mipGap(0.0),
        metadata(problem.getMetadata()),
        problemInstance(problem)
    {}

    // --- Management Methods ---
    void addRoute(int vehicleId, const VehicleRoute& route);

    void calculateViolations();

    // --- Visualization Methods ---
    void displaySummary();
    
    // --- Export Methods ---
    
    // Saves in a simple JSON format (ideal for external parsers)
    void saveToJSON(const std::string& filename) const;

    // Saves in a readable text format (report-like)
    void saveToTxt(const std::string& filename) const;

private:
    std::string generateSummaryString() const;
};
