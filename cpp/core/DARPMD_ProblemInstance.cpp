#include <iostream>
#include <fstream>
#include <iomanip>

#include "DARPMD_ProblemInstance.h"

void DARPMD_ProblemInstance::clear() {
    N_requests = 0;
    max_node_id = 0;
    P.clear();
    D.clear();
    K.clear();
    StartNode.clear();
    EndNode.clear();
    service_time.clear();
    demand.clear();
    time_window_start.clear();
    time_window_end.clear();
    capacity.clear();
    max_route_time.clear();
    max_ride_time = 0.0;
    t_ij.clear();
    c_ijk.clear();
    metadata = Metadata();
}

bool DARPMD_ProblemInstance::loadFromJSON(const std::string& path) {
    clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << path << std::endl;
        return false;
    }

    json j;
    try {
        file >> j;

        std::cout << "Cargando instancia..." << std::endl;

        // 1. Load Requests and basic Sets
        auto& req = j.at("requests");
        P = req.at("pickup_ids").get<std::vector<int>>();
        D = req.at("delivery_ids").get<std::vector<int>>();
        N_requests = req.at("n_requests");
        K_vehicles = j.at("vehicles").size();

        // Calculate the maximum node ID for resizing vectors
        max_node_id = 0;
        for (auto& n : j.at("nodes")) {
            int id = n.at("id");
            if (id > max_node_id) max_node_id = id;
        }

        // 2. Load Nodes
        for (auto& n : j.at("nodes")) {
            int id = n.at("id");
            service_time[id] = n.at("service_time");
            demand[id] = n.at("demand");
            time_window_start[id] = n.at("tw_start");
            time_window_end[id] = n.at("tw_end");
        }

        // 3. Load Vehicles
        for (auto& v : j.at("vehicles")) {
            int k = v.at("id");
            K.push_back(k);
            StartNode[k] = v.at("start_node");
            EndNode[k] = v.at("end_node");
            capacity[k] = v.at("capacity");
            max_route_time[k] = v.at("max_time");
        }

        // 4. Global Parameters
        max_ride_time = j.at("global_params").at("L_ride");

        // 5. Time Matrix (t_ij)
        t_ij.resize(max_node_id + 1, std::vector<double>(max_node_id + 1, 999999.0)); // Initalize with large time

        for (auto& x : j.at("matrix_t")) {
            int u = x.at("from");
            int v = x.at("to");
            double val = x.at("value");
            if (u <= max_node_id && v <= max_node_id) {
                t_ij[u][v] = val;
            }
        }

        // 6. Costs (c_ijk)
        for (auto& x : j.at("matrix_c")) {
            int u = x.at("from");
            int v = x.at("to");
            int k = x.at("k");
            double val = x.at("value");
            c_ijk[{u, v, k}] = val;
        }

        // 7. Load city metadata if present
        if (j.contains("metadata")) {
            metadata = j.at("metadata").get<Metadata>();
        }

    } catch (const json::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return false;
    }

    return true;
}

double DARPMD_ProblemInstance::getTravelTime(int i, int j) const {
    return t_ij[i][j];
}

double DARPMD_ProblemInstance::getCost(int i, int j, int k) const {
    auto it = c_ijk.find({i, j, k});
    if (it != c_ijk.end()) {
        return it->second;
    }
    std::cout << "Warning: Cost not found for (" << i << ", " << j << ", " << k << "). Returning 0.0." << std::endl;
    return 0.0;
}

double DARPMD_ProblemInstance::getServiceTime(int i) const {
    return service_time.at(i);
}

double DARPMD_ProblemInstance::getDemand(int i) const {
    return demand.at(i);
}

double DARPMD_ProblemInstance::getTimeWindowStart(int i) const {
    return time_window_start.at(i);
}

double DARPMD_ProblemInstance::getTimeWindowEnd(int i) const {
    return time_window_end.at(i);
}

double DARPMD_ProblemInstance::getVehicleCapacity(int k) const {
    return capacity.at(k);
}

double DARPMD_ProblemInstance::getVehicleMaxRouteTime(int k) const {
    return max_route_time.at(k);
}

void DARPMD_ProblemInstance::displayInfo() const {
    std::cout << "\n============================================\n";
    std::cout << "       DARPMD Problem Instance Info         \n";
    std::cout << "============================================\n";

    // --- Global Parameters ---
    std::cout << "--- Global Parameters ---\n";
    std::cout << std::left << std::setw(25) << "Number of Requests:" << N_requests << "\n";
    std::cout << std::left << std::setw(25) << "Max Node ID:" << max_node_id << "\n";
    std::cout << std::left << std::setw(25) << "Max Ride Time (L):" << max_ride_time << "\n";
    std::cout << "\n";

    // --- Sets ---
    std::cout << "--- Sets ---\n";
    auto printVector = [](const std::string& name, const std::vector<int>& v) {
        std::cout << name << ": [ ";
        for (const auto& item : v) std::cout << item << " ";
        std::cout << "] (Size: " << v.size() << ")\n";
    };

    printVector("Pickups (P)", P);
    printVector("Deliveries (D)", D);
    printVector("Vehicles (K)", K);
    std::cout << "\n";

    // --- Vehicle Details ---
    std::cout << "--- Vehicle Configuration (K) ---\n";
    std::cout << std::left 
              << std::setw(10) << "Veh_ID" 
              << std::setw(12) << "Start_Node" 
              << std::setw(12) << "End_Node" 
              << std::setw(12) << "Capacity" 
              << "Max_Route_Time" << "\n";
    std::cout << std::string(60, '-') << "\n";

    // --- Print vehicle details ---
    for (int k : K) {
        int start = (StartNode.count(k)) ? StartNode.at(k) : -1;
        int end = (EndNode.count(k)) ? EndNode.at(k) : -1;
        double cap = (capacity.count(k)) ? capacity.at(k) : 0.0;
        double max_t = (max_route_time.count(k)) ? max_route_time.at(k) : 0.0;

        std::cout << std::left 
                  << std::setw(10) << k 
                  << std::setw(12) << start 
                  << std::setw(12) << end 
                  << std::setw(12) << cap 
                  << max_t << "\n";
    }
    std::cout << "\n";

    // --- Node Attributes ---
    std::cout << "--- Node Attributes (First 15 or relevant) ---\n";
    std::cout << std::left 
              << std::setw(10) << "Node_ID" 
              << std::setw(15) << "Service_T" 
              << std::setw(10) << "Demand" 
              << std::setw(15) << "TW_Start" 
              << "TW_End" << "\n";
    std::cout << std::string(60, '-') << "\n";

    // Helper to print a node row
    auto printNodeRow = [&](int i) {
        // Bounds check
        if (i < 0 || i >= service_time.size()) return;
        
        std::cout << std::left 
                  << std::setw(10) << i 
                  << std::setw(15) << service_time.at(i)
                  << std::setw(10) << demand.at(i)
                  << std::setw(15) << time_window_start.at(i)
                  << time_window_end.at(i) << "\n";
    };

    // Print subset to avoid huge lists, or iterate relevant sets
    std::vector<int> nodes_to_show;
    // Add depots
    for(auto const& [k, node] : StartNode) nodes_to_show.push_back(node);
    for(auto const& [k, node] : EndNode) nodes_to_show.push_back(node);
    // Add first few Pickups and Deliveries as sample
    for(size_t i=0; i < P.size() && i < 5; ++i) nodes_to_show.push_back(P[i]);
    for(size_t i=0; i < D.size() && i < 5; ++i) nodes_to_show.push_back(D[i]);

    // Simple display of relevant nodes (or loop 0..max_node_id if prefered)
    for(int n : nodes_to_show) {
        printNodeRow(n);
    }
    if (P.size() > 5) std::cout << "... (and " << (P.size() + D.size() - 10) << " more P/D nodes)\n";
    std::cout << "\n";

    // --- Matrices Dimensions ---
    std::cout << "--- Matrices Dimensions ---\n";
    
    // Travel Time Matrix (t_ij)
    if (!t_ij.empty()) {
        std::cout << "Travel Time Matrix (t_ij): " 
                  << t_ij.size() << " rows x " 
                  << t_ij[0].size() << " cols\n";
    } else {
        std::cout << "Travel Time Matrix (t_ij): [Empty]\n";
    }

    // Cost Matrix (c_ijk)
    std::cout << "Cost Map (c_ijk): " << c_ijk.size() << " entries loaded.\n";

    std::cout << "============================================\n\n";
}