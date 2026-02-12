#pragma once

#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <utility>

#include "Metadata.h"

// We use the nlohmann/json library for JSON parsing
// https://github.com/nlohmann/json
#include "../includes/json.hpp" 
using json = nlohmann::json;


class DARPMD_ProblemInstance {
public:
    DARPMD_ProblemInstance() = default;
    
    void clear();
    bool loadFromJSON(const std::string& filename);
    void displayInfo() const;

    double getTravelTime(int i, int j) const;
    double getCost(int i, int j, int k) const;
    double getServiceTime(int i) const;
    double getDemand(int i) const;
    double getTimeWindowStart(int i) const;
    double getTimeWindowEnd(int i) const;
    double getVehicleCapacity(int k) const;
    double getVehicleMaxRouteTime(int k) const;

    // O(1) checks using preprocessed node status
    bool isPickup(int nodeId) const;
    bool isDelivery(int nodeId) const;
    bool isVehicleStart(int nodeId) const;
    bool isVehicleEnd(int nodeId) const;

    int N_requests; // Number of requests
    int K_vehicles; // Number of vehicles
    int max_node_id; // For sizing vectors

    // Sets
    std::vector<int> P; // Pickups
    std::vector<int> D; // Deliveries
    std::vector<int> K; // Vehicles
    
    // Vehicle mappings
    std::unordered_map<int, int> StartNode; // k -> start_node
    std::unordered_map<int, int> EndNode;   // k -> end_node
    
    // Node parameters
    std::unordered_map<int, double> service_time;           // d_i
    std::unordered_map<int, double> demand;                 // q_i
    std::unordered_map<int, double> time_window_start;      // e_i
    std::unordered_map<int, double> time_window_end;        // l_i

    // Vehicle parameters
    std::unordered_map<int, double> capacity;             // Q_k
    std::unordered_map<int, double> max_route_time;       // T_k

    // Global parameters
    double max_ride_time; // L

    // Time matrix (t_ij): vector of vectors
    std::vector<std::vector<double>> t_ij;

    // Costs matrix (c_ijk): map of tuples to double
    std::map<std::tuple<int, int, int>, double> c_ijk; // O(1) average, n is the number of triplets

    // --- City metadata ---
    Metadata metadata;
    const Metadata& getMetadata() const { return metadata; }

private:
    std::vector<int> allNodesStatus; // 0=pickup, 1=delivery, 2=depotStart, 3=depotEnd
};
