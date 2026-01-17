#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

#include "DARPMD_ResultInstance.h"

void DARPMD_ResultInstance::addRoute(int vehicleId, const VehicleRoute& route) {
    routes[vehicleId] = route;
}

std::string DARPMD_ResultInstance::generateSummaryString() const {
    std::stringstream ss;

    ss << "\n=== DARPMD SOLUTION SUMMARY ===\n";
    ss << "Status: " << solverStatus << "\n";
    ss << "Objective Value: " << objectiveValue << "\n";
    ss << "Compute Time: " << solveTime << " s\n";
    ss << "MIP Gap: " << mipGap << "\n";
    ss << "--------------------------------\n\n";
    
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

void DARPMD_ResultInstance::displaySummary() const {
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