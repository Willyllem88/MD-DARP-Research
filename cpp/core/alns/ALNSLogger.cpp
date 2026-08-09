#include "ALNSLogger.h"
#include <fstream>
#include <iostream>
#include <limits>

ALNSLogger::ALNSLogger(size_t expectedIterations, size_t expectedSegments) {
    iterationHistory.reserve(expectedIterations);
    weightsHistory.reserve(expectedSegments);
}

void ALNSLogger::recordIteration(int iter, double time, double currObj, double bestObj, 
                                 double bestFeasibleObj, int destOp, int repOp, 
                                 double reward, double temp, int segment) {
    iterationHistory.push_back({iter, time, currObj, bestObj, bestFeasibleObj, destOp, repOp, reward, temp, segment});
}

void ALNSLogger::recordWeights(int segment, double time, 
                               const std::vector<double>& destWeights, 
                               const std::vector<double>& repWeights) {
    weightsHistory.push_back({segment, time, destWeights, repWeights});
}

void ALNSLogger::exportConvergenceCSV(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir " << filepath << " para escribir el log." << std::endl;
        return;
    }

    // Cabeceras
    file << "iteration,elapsed_time,current_obj,best_obj,best_feasible_obj,destroy_op,repair_op,reward,temperature,segment\n";

    for (const auto& r : iterationHistory) {
        file << r.iteration << ","
             << r.elapsedTime << ","
             << r.currentObjective << ","
             << r.bestObjective << ",";
        
        // Manejar el caso de que no haya solución factible todavía
        if (r.bestFeasibleObjective == std::numeric_limits<double>::infinity()) {
            file << "inf,";
        } else {
            file << r.bestFeasibleObjective << ",";
        }

        file << r.destroyOpIdx << ","
             << r.repairOpIdx << ","
             << r.reward << ","
             << r.temperature << ","
             << r.segment << "\n";
    }
    file.close();
}

void ALNSLogger::exportWeightsEvolutionCSV(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open() || weightsHistory.empty()) return;

    // Crear la cabecera dinámicamente según la cantidad de operadores
    file << "segment,elapsed_time";
    for (size_t i = 0; i < weightsHistory[0].destroyWeights.size(); ++i) {
        file << ",destroy_weight_" << i;
    }
    for (size_t i = 0; i < weightsHistory[0].repairWeights.size(); ++i) {
        file << ",repair_weight_" << i;
    }
    file << "\n";

    // Escribir los datos
    for (const auto& w : weightsHistory) {
        file << w.segment << "," << w.elapsedTime;
        for (double val : w.destroyWeights) file << "," << val;
        for (double val : w.repairWeights) file << "," << val;
        file << "\n";
    }
    file.close();
}
