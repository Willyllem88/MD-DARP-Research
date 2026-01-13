#pragma once

#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <utility>

// We use the nlohmann/json library for JSON parsing
// https://github.com/nlohmann/json
#include "../includes/json.hpp" 
using json = nlohmann::json;

class DARPMD_ProblemInstance {
public:
    DARPMD_ProblemInstance() = default;
    bool loadFromJSON(const std::string& filename);
    void displayInfo() const;

    // Getters
    double getTravelTime(int i, int j) const;
    double getCost(int i, int j, int k) const;
    double getServiceTime(int i) const;
    double getDemand(int i) const;
    double getTimeWindowStart(int i) const;
    double getTimeWindowEnd(int i) const;
    double getVehicleCapacity(int k) const;
    double getVehicleMaxRouteTime(int k) const;

    int N_requests; // Number of requests
    int max_node_id; // For sizing vectors

    // Sets
    std::vector<int> P; // Pickups
    std::vector<int> D; // Deliveries
    std::vector<int> K; // Vehicles
    
    // Vehicle mappings
    std::map<int, int> StartNode; // k -> start_node
    std::map<int, int> EndNode;   // k -> end_node
    
    // Node parameters
    std::map<int, double> service_time;           // d_i
    std::map<int, double> demand;                 // q_i
    std::map<int, double> time_window_start;      // e_i
    std::map<int, double> time_window_end;        // l_i

    // Vehicle parameters
    std::map<int, double> capacity;             // Q_k
    std::map<int, double> max_route_time;       // T_k

    // Global parameters
    double max_ride_time; // L

    // Time matrix (t_ij): vector of vectors
    std::vector<std::vector<double>> t_ij;

    // Costs matrix (c_ijk): map of tuples to double
    std::map<std::tuple<int, int, int>, double> c_ijk;
};
