#include <iostream>

#include "DARPMD_ProblemInstance.h"
#include "DARPMDSolver.h"

const std::string DEFAULT_INSTANCE_PATH = "/home/guillem/TFG-Guillem/data/instances_static/gracia-4R2V.json";

//TODO: Add command line argument parsing for instance file and time limit
int main() {
    DARPMD_ProblemInstance instance;
    if (!instance.loadFromJSON(DEFAULT_INSTANCE_PATH)) {
        return 1; // Error loading instance
    }
    instance.displayInfo();

    DARPMDSolver solver(instance);
    solver.solve(3600.0);
    solver.displayResults();

    return 0;
}