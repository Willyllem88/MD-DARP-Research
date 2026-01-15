#include "DARPMD_ResultInstance.h"

void DARPMD_ResultInstance::addRoute(int vehicleId, const VehicleRoute& route) {
    routes[vehicleId] = route;
}

void DARPMD_ResultInstance::displaySummary() const {
    std::cout << "\n=== DARPMD SOLUTION SUMMARY ===\n";
    std::cout << "Status: " << solverStatus << "\n";
    std::cout << "Objective Value: " << objectiveValue << "\n";
    std::cout << "Compute Time: " << solutionTimeSec << " s\n";
    std::cout << "--------------------------------\n\n";

    auto nodeName = [](int i, int numReq, int numVeh) -> std::string {
        // 123456
        // 1+
        // 2+
        // 1-
        // 2-
        // V0Start
        // V0End
        if (i <= numReq) return std::to_string(i)+"+";
        if (i <= 2*numReq) return std::to_string(i - numReq)+"-";
        if (i < 2*numReq + numVeh) return "V" + std::to_string(i - 2*numReq) + "St";
        return "V" + std::to_string(i - 2*numReq - numVeh) + "En";
    };

    for (const auto& [k, route] : routes) {
        // Header for each vehicle
        std::cout << "Vehicle " << k << ": ";
        
        if (route.isEmpty()) {
            std::cout << "UNUSED (Stays at Depot)\n\n";
            continue;
        }

        std::cout << "Route (Steps: " << route.steps.size() << ")\n";
        
        // Column headers
        std::cout << "  " // Light indentation
                  << std::left << std::setw(10) << "NodeID"
                  << std::setw(15) << "NodeName"
                  << std::setw(15) << "Type" 
                  << std::setw(12) << "ArrTime" 
                  << std::setw(12) << "Load" << "\n";

        std::cout << "  " << std::string(49, '-') << "\n"; // Visual separator

        // Data rows
        for (const auto& step : route.steps) {
            std::cout << "  " // Light indentation
                      << std::left << std::setw(10) << step.nodeId
                      << std::setw(15) << nodeName(step.nodeId, problemInstance.N_requests, problemInstance.K_vehicles)
                      << std::setw(15) << step.type 
                      << std::setw(12) << std::fixed << std::setprecision(2) << step.arrivalTime 
                      << std::setw(12) << std::setprecision(1) << step.loadAfter << "\n";
        }
        std::cout << "\n";
    }
    std::cout << "===============================\n" << std::endl;
}

void DARPMD_ResultInstance::saveToTxt(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << filename << " para escribir." << std::endl;
        return;
    }

    file << "=== DARPMD SOLUTION REPORT ===\n";
    file << "Status: " << solverStatus << "\n";
    file << "Objective Value: " << objectiveValue << "\n";
    file << "Compute Time: " << solutionTimeSec << " s\n";
    file << "--------------------------------\n\n";

    for (const auto& [k, route] : routes) {
        if (route.isEmpty()) {
            file << "Vehicle " << k << ": UNUSED\n";
            continue;
        }

        file << "Vehicle " << k << " Route (Steps: " << route.steps.size() << ")\n";
        file << std::left << std::setw(10) << "NodeID" 
             << std::setw(15) << "Type" 
             << std::setw(12) << "ArrTime" 
             << std::setw(12) << "Load" << "\n";

        for (const auto& step : route.steps) {
            file << std::left << std::setw(10) << step.nodeId 
                 << std::setw(15) << step.type 
                 << std::setw(12) << std::fixed << std::setprecision(2) << step.arrivalTime 
                 << std::setw(12) << std::setprecision(1) << step.loadAfter << "\n";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Report saved to: " << filename << std::endl;
}

void DARPMD_ResultInstance::saveToJSON(const std::string& filename) const {
    // 1. Crear el objeto JSON raíz
    json root;

    // 2. Rellenar Summary
    root["summary"] = {
        {"status", solverStatus},
        {"objective", objectiveValue},
        {"time_sec", solutionTimeSec}
    };

    // 3. Rellenar Routes (Construcción automática de arrays)
    root["routes"] = json::array();
    for (const auto& [k, route] : routes) {
        json routeJson;
        routeJson["vehicle_id"] = k;
        
        // Transformar vector de C++ a array JSON
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

    // 4. Lógica de Metadata (SOLO SI NO ESTÁ VACÍA)
    // Aquí usamos el método que creamos en el paso 1
    if (!metadata.isEmpty()) {
        root["metadata"] = metadata; 
        // La librería maneja la coma anterior y el formato automáticamente
    }

    // 5. Guardar en fichero
    std::ofstream file(filename);
    if (file.is_open()) {
        file << std::setw(4) << root << std::endl; // setw(4) lo deja bonito (pretty print)
        std::cout << "JSON saved to: " << filename << std::endl;
    }
}