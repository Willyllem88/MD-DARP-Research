#include "InstanceAnalyzer.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

InstanceAnalyzer::InstanceAnalyzer(const DARPMD_ProblemInstance& instance) 
    : data(instance) {}

void InstanceAnalyzer::printHeader(const std::string& title) const {
    std::cout << "\n========================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "========================================\n";
}

void InstanceAnalyzer::printStatLine(const std::string& label, const StatResult& stat) const {
    std::cout << std::left << std::setw(25) << label 
              << " | Min: " << std::setw(8) << stat.min 
              << " | Max: " << std::setw(8) << stat.max 
              << " | Avg: " << stat.avg << "\n";
}

InstanceAnalyzer::StatResult InstanceAnalyzer::calculateTimeWindowStats() const {
    double min_w = std::numeric_limits<double>::max();
    double max_w = 0;
    double sum_w = 0;
    int count = 0;

    // Analyze TW for Requests (Pickups and Deliveries), excluding Depots
    auto analyze_node = [&](int node_id) {
        double width = data.getTimeWindowEnd(node_id) - data.getTimeWindowStart(node_id);
        if (width < min_w) min_w = width;
        if (width > max_w) max_w = width;
        sum_w += width;
        count++;
    };

    for (int i : data.P) analyze_node(i);
    for (int i : data.D) analyze_node(i);

    return {min_w, max_w, (count > 0 ? sum_w / count : 0.0), sum_w};
}

InstanceAnalyzer::StatResult InstanceAnalyzer::calculateDemandStats() const {
    double min_q = std::numeric_limits<double>::max();
    double max_q = 0;
    double sum_q = 0;
    int count = 0;

    for (int i : data.P) {
        double q = data.getDemand(i);
        if (q < min_q) min_q = q;
        if (q > max_q) max_q = q;
        sum_q += q;
        count++;
    }

    return {min_q, max_q, (count > 0 ? sum_q / count : 0.0), sum_q};
}

double InstanceAnalyzer::calculateGlobalCapacityTightness() const {
    double total_vehicle_capacity = 0;
    for (int k : data.K) {
        total_vehicle_capacity += data.getVehicleCapacity(k);
    }
    
    StatResult demandStats = calculateDemandStats();
    
    if (total_vehicle_capacity == 0) return 0.0;
    return (demandStats.sum / total_vehicle_capacity) * 100.0;
}

void InstanceAnalyzer::printReport() const {
    printHeader("DARPMD INSTANCE REPORT");

    // 1. General Dimensions
    std::cout << "Requests (n):  " << data.N_requests << "\n";
    std::cout << "Vehicles (k):  " << data.K_vehicles << "\n";
    std::cout << "Max Ride Time (L): " << data.getMaxRideTime() << "\n";

    // 2. Time Window Analysis
    // Cordeau & Laporte (2003) emphasize TW width as a hardness factor.
    printHeader("Time Windows (TW)");
    StatResult twStats = calculateTimeWindowStats();
    printStatLine("TW Width [l_i - e_i]", twStats);
    
    // 3. Demand vs Capacity
    printHeader("Capacity & Demand");
    StatResult demStats = calculateDemandStats();
    printStatLine("Request Demand", demStats);
    
    double tightness = calculateGlobalCapacityTightness();
    std::cout << "Global Capacity Utilization: " << std::fixed << std::setprecision(2) << tightness << "%\n";
    std::cout << "(Total Demand / Sum of All Vehicle Capacities)\n";

    // 4. Vehicle Heterogeneity
    // Check if vehicles have different capacities or max times
    bool hetero_cap = false;
    bool hetero_time = false;
    double first_cap = data.getVehicleCapacity(data.K[0]);
    double first_time = data.getVehicleMaxRouteTime(data.K[0]);

    for(int k : data.K) {
        if(std::abs(data.getVehicleCapacity(k) - first_cap) > 0.001) hetero_cap = true;
        if(std::abs(data.getVehicleMaxRouteTime(k) - first_time) > 0.001) hetero_time = true;
    }

    printHeader("Fleet Characteristics");
    std::cout << "Heterogeneous Capacity: " << (hetero_cap ? "Yes" : "No") << "\n";
    std::cout << "Heterogeneous MaxTime:  " << (hetero_time ? "Yes" : "No") << "\n";
    if (!hetero_cap) std::cout << "Uniform Capacity:       " << first_cap << "\n";
    if (!hetero_time) std::cout << "Uniform Max Route Time: " << first_time << "\n";

    std::cout << "\n[End of Report]\n";
}