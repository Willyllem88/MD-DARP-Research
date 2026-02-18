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

    // Getters for node attributes (optimized for direct access)
    inline double getServiceTime(int i) const { return service_time[i]; }
    inline double getDemand(int i) const { return demand[i]; }
    inline double getTimeWindowStart(int i) const { return time_window_start[i]; }
    inline double getTimeWindowEnd(int i) const { return time_window_end[i]; }

    // Getters for vehicle attributes (optimized for direct access)
    inline double getVehicleCapacity(int k) const { return capacity[k]; }
    inline double getVehicleMaxRouteTime(int k) const { return max_route_time[k]; }

    inline double getTravelTime(int i, int j) const { return t_ij[i * stride_time_i + j]; }
    inline double getCost(int i, int j, int k) const { 
        size_t index = (size_t)i * stride_cost_i + (size_t)j * stride_cost_j + (size_t)k;
        return flat_cost_matrix[index];
    }

    inline double getMaxRideTime() const { return max_ride_time; }

    // Logic to determine node type based on ID
    // P: 1..n | D: n+1..2n | DepStart: 2n+1..2n+k | DepEnd: 2n+k+1..2n+2k
    inline bool isPickup(int i) const { 
        return i >= 1 && i <= N_requests; 
    }
    inline bool isDelivery(int i) const { 
        return i > N_requests && i <= 2 * N_requests; 
    }
    inline bool isVehicleStart(int i) const { 
        return i > 2 * N_requests && i <= 2 * N_requests + K_vehicles;
    }
    inline bool isVehicleEnd(int i) const { 
        return i > 2 * N_requests + K_vehicles && i <= max_node_id;
    }

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

    // --- City metadata ---
    const Metadata& getMetadata() const { return metadata; }

    // Triangle Inequality Check and Fix (for travel times and costs) [UNUSED]
    void checkAndFixTriangleInequality(bool fixIt = false, bool verbose = true);

private:
    std::vector<double> service_time;           // d_i
    std::vector<double> demand;                 // q_i
    std::vector<double> time_window_start;      // e_i
    std::vector<double> time_window_end;        // l_i

    std::vector<double> capacity;            // Q_k
    std::vector<double> max_route_time;      // T_k

    std::vector<double> t_ij;   // t_ij
    size_t stride_time_i = 0;

    std::vector<double> flat_cost_matrix;
    size_t stride_cost_i = 0;
    size_t stride_cost_j = 0;

    double max_ride_time;                    // L

    Metadata metadata; // City metadata

    double INF = 1e9; // A large number to represent infinity for time/cost
};
