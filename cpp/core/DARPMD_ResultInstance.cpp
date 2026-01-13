#include "DARPMD_ResultInstance.h"

DARPMD_ResultInstance::DARPMD_ResultInstance() 
    : objectiveValue(0.0), solverStatus("Unknown"), solutionTimeSec(0.0) {}

void DARPMD_ResultInstance::addRoute(int vehicleId, const VehicleRoute& route) {
    routes[vehicleId] = route;
}

void DARPMD_ResultInstance::displaySummary() const {
    std::cout << "\n=== DARPMD SOLUTION SUMMARY ===\n";
    std::cout << "Status: " << solverStatus << "\n";
    std::cout << "Objective Value: " << objectiveValue << "\n";
    std::cout << "Compute Time: " << solutionTimeSec << " s\n";
    std::cout << "--------------------------------\n\n";

    for (const auto& [k, route] : routes) {
        // Encabezado del vehículo
        std::cout << "Vehicle " << k << ": ";
        
        if (route.isEmpty()) {
            std::cout << "UNUSED (Stays at Depot)\n\n";
            continue;
        }

        std::cout << "Route (Steps: " << route.steps.size() << ")\n";
        
        // Encabezados de columna
        std::cout << "  " // Indentación ligera
                  << std::left << std::setw(10) << "NodeID" 
                  << std::setw(15) << "Type" 
                  << std::setw(12) << "ArrTime" 
                  << std::setw(12) << "Load" << "\n";

        std::cout << "  " << std::string(49, '-') << "\n"; // Separador visual

        // Filas de datos
        for (const auto& step : route.steps) {
            std::cout << "  " // Indentación ligera
                      << std::left << std::setw(10) << step.nodeId 
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
    std::ofstream file(filename);
    if (!file.is_open()) return;

    // Construcción manual de JSON
    file << "{\n";
    file << "  \"summary\": {\n";
    file << "    \"status\": \"" << solverStatus << "\",\n";
    file << "    \"objective\": " << objectiveValue << ",\n";
    file << "    \"time_sec\": " << solutionTimeSec << "\n";
    file << "  },\n";
    file << "  \"routes\": [\n";

    bool firstRoute = true;
    for (const auto& [k, route] : routes) {
        if (!firstRoute) file << ",\n";
        firstRoute = false;

        file << "    {\n";
        file << "      \"vehicle_id\": " << k << ",\n";
        file << "      \"steps\": [\n";
        
        for (size_t i = 0; i < route.steps.size(); ++i) {
            const auto& step = route.steps[i];
            file << "        { \"node\": " << step.nodeId 
                 << ", \"type\": \"" << step.type << "\""
                 << ", \"arrival\": " << step.arrivalTime 
                 << ", \"load\": " << step.loadAfter << " }";
            
            if (i < route.steps.size() - 1) file << ",";
            file << "\n";
        }
        file << "      ]\n";
        file << "    }";
    }
    file << "\n  ]\n";
    file << "}\n";
    
    file.close();
    std::cout << "JSON saved to: " << filename << std::endl;
}