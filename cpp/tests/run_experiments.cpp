#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <memory>

#include "DARPMD_ProblemInstance.h"
#include "CPLEXSolver.h"

namespace fs = std::filesystem;

int main() {
    std::string instances_dir = "data/instances_static/cordeau-instances";
    std::string output_csv = "results.csv";
    
    double time_limit = 7200.0; 

    std::ofstream csv_file(output_csv);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening or creating CSV file: " << output_csv << std::endl;
        return 1;
    }

    csv_file << "Instance,ObjectiveValue,SolverStatus,SolveTime,MIPGap\n";

    if (!fs::exists(instances_dir) || !fs::is_directory(instances_dir)) {
        std::cerr << "Error: directory " << instances_dir << " not found." << std::endl;
        std::cerr << "Make sure to execute the program from the root of the project." << std::endl;
        return 1;
    }

    std::vector<std::string> test_instances = {
        "a2-16.json", "a2-20.json", "a2-24.json", "a3-18.json",
        "a3-24.json", "a3-30.json", "a3-36.json", "a4-16.json",
        "a4-24.json", "a4-32.json", "a4-40.json", "a4-48.json"
    };

    for (const auto& filename : test_instances) {
        std::string filepath = instances_dir + "/" + filename;

        if (!fs::exists(filepath)) {
            std::cerr << "Warning: File not found " << filepath << ". Skipping..." << std::endl;
            continue;
        }

        std::cout << "----------------------------------------\n";
        std::cout << "Solving instance: " << filename << std::endl;

        DARPMD_ProblemInstance instance;
        if (!instance.loadFromJSON(filepath)) {
            std::cerr << "Failed to load instance: " << filepath << std::endl;
            continue;
        }

        auto solver = std::make_unique<CPLEXSolver>(instance, time_limit);
        
        solver->solve();
        DARPMD_ResultInstance result = solver->getResult();

        std::cout << "Status: " << result.solverStatus 
                    << " | Objective: " << result.objectiveValue 
                    << " | Time: " << result.solveTime << "s" << std::endl;

        csv_file << filename << ","
                    << result.objectiveValue << ","
                    << result.solverStatus << ","
                    << result.solveTime << ","
                    << result.mipGap << "\n";

        csv_file.flush(); 
    }

    csv_file.close();
    std::cout << "----------------------------------------\n";
    std::cout << "All experiments have finished. Results saved in " << output_csv << std::endl;

    return 0;
}