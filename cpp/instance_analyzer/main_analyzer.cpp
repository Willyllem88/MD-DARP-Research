#include <iostream>
#include <string>
#include <vector>
#include <cstring> // for strcmp

#include "../core/DARPMD_ProblemInstance.h"
#include "InstanceAnalyzer.h"

void printUsage(const char* progName) {
    std::cerr << "Usage: " << progName << " [-i <path_to_json_instance>]\n";
}

int main(int argc, char* argv[]) {
    std::string instancePath = "";

    // 1. Argument Parsing
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-i") == 0) {
            if (i + 1 < argc) {
                instancePath = argv[i + 1];
                i++; // Skip next arg
            } else {
                std::cerr << "Error: -i option requires a file path.\n";
                printUsage(argv[0]);
                return 1;
            }
        }
    }

    // 2. Interactive Fallback
    if (instancePath.empty()) {
        std::cout << "No instance file provided via arguments.\n";
        std::cout << "Please enter the path to the JSON instance file: ";
        std::cin >> instancePath;
    }

    std::cout << "Loading instance: " << instancePath << " ...\n";

    // 3. Load Instance
    DARPMD_ProblemInstance problem;
    try {
        bool success = problem.loadFromJSON(instancePath);
        if (!success) {
            std::cerr << "Failed to load instance (loadFromJSON returned false).\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception while loading instance: " << e.what() << "\n";
        return 1;
    }

    // 4. Run Analysis
    try {
        InstanceAnalyzer analyzer(problem);
        analyzer.printReport();
    } catch (const std::exception& e) {
        std::cerr << "Error during analysis: " << e.what() << "\n";
        return 1;
    }

    return 0;
}