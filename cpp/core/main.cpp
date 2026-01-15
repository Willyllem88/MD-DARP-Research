#include <iostream>
#include <string>
#include <optional>

#include "DARPMD_ProblemInstance.h"
#include "DARPMDSolver.h"

const std::string DEFAULT_INSTANCE_PATH = "/home/guillem/TFG-Guillem/data/instances_static/gracia-4R2V.json";


struct Args {
    std::string instance_path = DEFAULT_INSTANCE_PATH;
    std::optional<double> time_limit;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-l" && i + 1 < argc) {
            args.instance_path = argv[++i];
        } else if (a == "-t" && i + 1 < argc) {
            args.time_limit = std::stod(argv[++i]);
        }
        else {
            std::cerr << "Unknown argument: " << a << std::endl;
        }
    }
    return args;
}

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    DARPMD_ProblemInstance instance;
    if (!instance.loadFromJSON(args.instance_path)) {
        std::cerr << "Failed to load instance from " << args.instance_path << std::endl;
        return 1;
    }
    instance.displayInfo();

    DARPMDSolver solver(instance);
    solver.solve(args.time_limit);

    DARPMD_ResultInstance result = solver.extractResult();
    result.displaySummary();
    result.saveToTxt("solution_report.txt");
    result.saveToJSON("solution_report.json");

    return 0;
}