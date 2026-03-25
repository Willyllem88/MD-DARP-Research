#include <iostream>
#include <fstream>
#include <iomanip>

#include "DARPMD_ProblemInstance.h"

void DARPMD_ProblemInstance::clear() {
    N_requests = 0;
    K_vehicles = 0;
    max_node_id = 0;
    P.clear();
    D.clear();
    K.clear();
    S.clear();
    E.clear();
    
    service_time.clear();
    demand.clear();
    time_window_start.clear();
    time_window_end.clear();
    capacity.clear();
    max_route_time.clear();
    max_ride_time = 0.0;

    t_ij.clear();
    flat_cost_matrix.clear();
    stride_time_i = 0;
    stride_cost_i = 0;
    stride_cost_j = 0;
    
    metadata = Metadata();
}

bool DARPMD_ProblemInstance::loadFromJSON(const std::string& path) {
    clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << path << std::endl;
        return false;
    }

    json j;
    try {
        file >> j;

        std::cout << "Loading instance..." << std::endl;

        // 1. Load Requests and basic Sets
        auto& req = j.at("requests");
        P = req.at("pickup_ids").get<std::vector<int>>();
        D = req.at("delivery_ids").get<std::vector<int>>();
        N_requests = req.at("n_requests");
        K_vehicles = j.at("vehicles").size();

        // Calculate the maximum node ID for resizing vectors
        max_node_id = 2 * N_requests +2 * K_vehicles;

        // Reserve memory
        service_time.resize(max_node_id + 1, 0.0);
        demand.resize(max_node_id + 1, 0.0);
        time_window_start.resize(max_node_id + 1, 0.0);
        time_window_end.resize(max_node_id + 1, 0.0);
        
        capacity.resize(K_vehicles + 1, 0.0);
        max_route_time.resize(K_vehicles + 1, 0.0);

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
            S.push_back(v.at("start_node"));
            E.push_back(v.at("end_node"));
            capacity[k] = v.at("capacity");
            max_route_time[k] = v.at("max_time");
        }

        // 4. Global Parameters
        max_ride_time = j.at("global_params").at("L_ride");

        // 5. Time Matrix (t_ij)
        t_ij.resize((max_node_id + 1) * (max_node_id + 1), INF); // Initialize with INF
        stride_time_i = max_node_id + 1;
        for (auto& x : j.at("matrix_t")) {
            int u = x.at("from");
            int v = x.at("to");
            size_t index = (size_t)u * stride_time_i + (size_t)v;

            if (index < t_ij.size()) {
                t_ij[index] = x.at("value");
            }
        }

        // 6. Costs (c_ijk)
        int dim_node = max_node_id + 1;
        int dim_veh = K_vehicles + 1;
        stride_cost_i = dim_node;
        stride_cost_j = 1;
        stride_cost_k = dim_node * dim_node;
        size_t total_size = (size_t)dim_node * dim_node * dim_veh;
        flat_cost_matrix.resize(total_size, INF);
        for (auto& x : j.at("matrix_c")) {
            int u = x.at("from");
            int v = x.at("to");
            int k = x.at("k");
            size_t index = (size_t)u * stride_cost_i + 
                           (size_t)v * stride_cost_j + 
                           (size_t)k * stride_cost_k;
            
            if (index < flat_cost_matrix.size()) {
                flat_cost_matrix[index] = x.at("value");
            }
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
    printVector("Start Depots (S)", S);
    printVector("End Depots (E)", E);
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
        int start = getVehicleStartNode(k);
        int end = getVehicleEndNode(k);
        double cap = capacity[k];
        double max_t = max_route_time[k];

        std::cout << std::left 
                  << std::setw(10) << k 
                  << std::setw(12) << start 
                  << std::setw(12) << end 
                  << std::setw(12) << cap 
                  << max_t << "\n";
    }
    std::cout << "\n";

    // --- Node Attributes ---
    std::cout << "--- Node Attributes ---\n";
    std::cout << std::left 
              << std::setw(10) << "Node_ID" 
              << std::setw(15) << "Service_T" 
              << std::setw(10) << "Demand" 
              << std::setw(15) << "TW_Start" 
              << "TW_End" << "\n";
    std::cout << std::string(60, '-') << "\n";

    // Helper to print a node row
    auto printNodeRow = [&](int i) {
        // Skip if node not defined
        if (i < 0 || i > max_node_id) return;

        std::cout << std::left 
                  << std::setw(10) << i 
                  << std::setw(15) << service_time.at(i)
                  << std::setw(10) << demand.at(i)
                  << std::setw(15) << time_window_start.at(i)
                  << time_window_end.at(i) << "\n";
    };

    // Print subset to avoid huge lists, or iterate relevant sets
    std::vector<int> all_nodes;
    // Add depots
    for(int s : S) all_nodes.push_back(s);
    for(int e : E) all_nodes.push_back(e);
    // Add pickup and deliveries
    for(size_t i=0; i < P.size(); ++i) {
        all_nodes.push_back(P[i]);
        all_nodes.push_back(D[i]);
    }

    for(int n : all_nodes) {
        printNodeRow(n);
    }
    std::cout << "\n";

    // --- Matrices Dimensions ---
    std::cout << "--- Matrices Dimensions ---\n";
    
    // Travel Time Matrix (t_ij)
    std::cout << "Travel Time Matrix (t_ij): " 
              << "Dimensions: (" << max_node_id + 1 << " x " << max_node_id + 1 << ")\n";

    // Cost Matrix (c_ijk)
    std::cout << "Cost Matrix (c_ijk): " 
              << "Dimensions: (" << max_node_id + 1 << " x " 
              << max_node_id + 1 << " x " 
              << K_vehicles + 1 << ")\n";

    std::cout << "============================================\n\n";
}

void DARPMD_ProblemInstance::checkAndFixTriangleInequality(bool fixIt, bool verbose) {
    // Check triangle inequality for travel times and costs
    const double TOLERANCE = 1e-4;
    int timeViolations = 0;
    double maxTimeViolation = 0.0;

    // Check triangle inequality for travel times (t_ij)
    if (!t_ij.empty()) {
        for (int k = 1; k <= max_node_id; ++k) { // middle node
            for (int i = 1; i <= max_node_id; ++i) { // origin
                if (i == k) continue;

                // Acceso directo optimizado
                size_t idx_i_base = (size_t)i * stride_time_i;
                double dist_ik = t_ij[idx_i_base + k]; // i->k

                // if i -> k is INF, it makes no sense to check detours through k, skip
                if (dist_ik >= INF) continue;

                for (int j = 1; j <= max_node_id; ++j) { // destination
                    if (j == i || j == k) continue;

                    size_t idx_k_j = (size_t)k * stride_time_i + j;
                    size_t idx_i_j = idx_i_base + j; // direct i -> j

                    double dist_kj = t_ij[idx_k_j];
                    double directDist = t_ij[idx_i_j];
                    double detourDist = dist_ik + dist_kj;

                    // Is it faster to go i->k->j than directly i->j?
                    if (directDist > detourDist + TOLERANCE) {
                        timeViolations++;
                        double diff = directDist - detourDist;
                        if (diff > maxTimeViolation) {
                            maxTimeViolation = diff;
                        }

                        if (fixIt) {
                            t_ij[idx_i_j] = detourDist;
                        }
                    }
                }
            }
        }
    }

    // Report results
    if (verbose) {
        if (timeViolations > 0) {
            std::cout << "[Triangle Inequality Violations] Time violations detected " << std::endl;
            std::cout << "  Number of violations: " << timeViolations << std::endl;
            std::cout << "  Max violation: " << maxTimeViolation << std::endl;
            
            if (fixIt) {
                std::cout << "[Triangle Inequality Violations] Time matrix have been fixed. Now, time triangle inequality holds." << std::endl;
            } else {
                std::cout << "[Triangle Inequality Violations] Some inconsistencies detected. It is recommended to use fixIt=true." << std::endl;
            }
        } else {
            std::cout << "[Triangle Inequality Violations] Times are consistent (Satisfy Triangle Inequality)." << std::endl;
        }
        std::cout << "--------------------------------------------------------------" << std::endl;
    }
}