#include <iostream>
#include <fstream>
#include <iomanip>

#include "DARPMD_ProblemInstance.h"

bool DARPMD_ProblemInstance::loadFromJSON(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << path << std::endl;
        return false;
    }

    json j;
    try {
        file >> j;
    } catch (const std::exception& e) {
        std::cerr << "Error parseando JSON: " << e.what() << std::endl;
        return false;
    }

    std::cout << "Cargando instancia..." << std::endl;

    // 1. Cargar Requests y Sets básicos
    auto& req = j["requests"];
    P = req["pickup_ids"].get<std::vector<int>>();
    D = req["delivery_ids"].get<std::vector<int>>();
    N_requests = req["n_requests"];

    // Calcular el ID de nodo máximo para redimensionar vectores
    max_node_id = 0;
    for (auto& n : j["nodes"]) {
        int id = n["id"];
        if (id > max_node_id) max_node_id = id;
    }

    // 2. Cargar Nodos
    for (auto& n : j["nodes"]) {
        int id = n["id"];
        service_time[id] = n["service_time"];
        demand[id] = n["demand"];
        time_window_start[id] = n["tw_start"];
        time_window_end[id] = n["tw_end"];
    }

    // 3. Cargar Vehículos
    for (auto& v : j["vehicles"]) {
        int k = v["id"];
        K.push_back(k);
        StartNode[k] = v["start_node"];
        EndNode[k] = v["end_node"];
        capacity[k] = v["capacity"];
        max_route_time[k] = v["max_time"];
    }

    // 4. Parámetros Globales
    // En tu python: data['L'] = json_data['global_params']['L_ride']
    max_ride_time = j["global_params"]["L_ride"];

    // 5. Matriz de Tiempos (t_ij)
    // Inicializar matriz NxN con ceros
    t_ij.resize(max_node_id + 1, std::vector<double>(max_node_id + 1, 0.0));

    for (auto& x : j["matrix_t"]) {
        int u = x["from"];
        int v = x["to"];
        double val = x["value"];
        if (u <= max_node_id && v <= max_node_id) {
            t_ij[u][v] = val;
        }
    }

    // 6. Costos (c_ijk)
    for (auto& x : j["matrix_c"]) {
        int u = x["from"];
        int v = x["to"];
        int k = x["k"];
        double val = x["value"];
        c_ijk[{u, v, k}] = val;
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
    return 0.0; // O algún valor por defecto
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

#include <iostream>
#include <iomanip>

/**
 * @brief Displays the complete summary of the loaded problem instance to the console.
 * * This function prints:
 * - Global parameters (Requests, Max Ride Time, etc.).
 * - The contents of the Sets P (Pickups), D (Deliveries), and K (Vehicles).
 * - Detailed attributes for each Vehicle (Start, End, Capacity, Max Time).
 * - Node attributes (Service Time, Demand, Time Windows) for relevant nodes.
 * - The dimensions (size) of the cost and time matrices (to avoid console flooding).
 * * @note Output text is in English.
 */
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

    for (int k : K) {
        // Using 'at' for safety, or find/end if logic implies missing keys
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
    // We iterate up to max_node_id to show loaded data. 
    // Usually, we filter this to show only relevant nodes (P U D U Depots), 
    // but here we check if data exists within bounds.
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