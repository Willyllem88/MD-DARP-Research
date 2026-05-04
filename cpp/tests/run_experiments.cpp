#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <memory>

#include "DARPMD_ProblemInstance.h"
#include "CPLEXSolver.h"
#include "CPLEXSoftSolver.h"

namespace fs = std::filesystem;

int main() {
    std::string instances_dir = "data/cordeau-instances";
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

        DARPMD_ProblemInstance instance;
        if (!instance.loadFromJSON(filepath)) {
            std::cerr << "Failed to load instance: " << filepath << std::endl;
            continue;
        }

        auto solver = std::make_unique<CPLEXSolver>(instance, time_limit);
        
        solver->solve();
        DARPMD_ResultInstance result = solver->getResult();

        std::cout << "Name: " << filename
                    << " | Status: " << result.solverStatus 
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

void experiment_softVShard(DARPMD_ProblemInstance& instance, std::string name) {
    struct Result {
        int num_variables;
        int num_constraints;
        double LP_bound;
    };

    auto hard_solver = std::make_unique<CPLEXSolver>(instance);
    auto soft_solver = std::make_unique<CPLEXSoftSolver>(instance);

    Result hard_result = {
        hard_solver->getNumberOfVariables(),
        hard_solver->getNumberOfConstraints(),
        hard_solver->solveLPRelaxation()
    };

    Result soft_result = {
        soft_solver->getNumberOfVariables(),
        soft_solver->getNumberOfConstraints(),
        soft_solver->solveLPRelaxation()
    };

    std::cout << name << " & "
                << instance.K_vehicles << " & "
                << instance.N_requests << " & "
                << instance.getVehicleMaxRouteTime(1) << " & "
                << instance.getVehicleCapacity(1) << " & "
                << instance.getMaxRideTime() << " & "
                << hard_result.num_constraints << " & "
                << hard_result.num_variables << " & "
                << hard_result.LP_bound << " & "
                << soft_result.num_constraints << " & "
                << soft_result.num_variables << " & "
                << soft_result.LP_bound << " &  \\\\  \n";
}