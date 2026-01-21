#include <iostream>
#include <string>
#include <optional>

#include "DARPMD_ProblemInstance.h"
#include "Solver.h"
#include "DARPMDSolver.h"
#include "DARPMDTabuSolver.h"

const std::string DEFAULT_INSTANCE_PATH = "/home/guillem/TFG-Guillem/data/instances_static/gracia-4R2V.json";


struct Args {
    std::string instance_path = DEFAULT_INSTANCE_PATH;
    std::string output_path = "solution_report.json";
    std::string method = "ILP"; // or "Tabu"
    std::optional<double> time_limit;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-i" || a == "--instance") && i + 1 < argc) {
            args.instance_path = argv[++i];
        } else if ((a == "-t" || a == "--time") && i + 1 < argc) {
            args.time_limit = std::stod(argv[++i]);
        }
        else if ((a == "-o" || a == "--output") && i + 1 < argc) {
            args.output_path = argv[++i];
        }
        else if ((a == "-m" || a == "--method") && i + 1 < argc) {
            std::string method = argv[++i];
            if (method != "ILP" && method != "Tabu") {
                std::cerr << "Unknown method: " << method << ". Use 'ILP' or 'Tabu'." << std::endl;
                exit(1);
            }
            args.method = method;
        }
        else if (a == "-h") {
            std::cout << "DARPMD Solver" << std::endl;
            std::cout << "Usage: " << argv[0] << " [-i instance_path] [-t time_limit] [-o output_path] [-m method]" << std::endl;
            std::cout << "  -i, --instance   Path to problem instance JSON file" << std::endl;
            std::cout << "  -t, --time       Time limit in seconds (optional)" << std::endl;
            std::cout << "  -o, --output     Path to output solution file" << std::endl;
            std::cout << "  -m, --method     Solver method: 'ILP' or 'Tabu'" << std::endl;
            std::cout << "  -h              Show this help message" << std::endl;
            exit(0);
        }
        else {
            std::cerr << "Unknown argument: " << a << std::endl;
        }
    }
    return args;
}

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    // Load Problem Instance
    DARPMD_ProblemInstance instance;
    if (!instance.loadFromJSON(args.instance_path)) {
        std::cerr << "Failed to load instance from " << args.instance_path << std::endl;
        return 1;
    }
    instance.displayInfo();

    // Select Solver
    Solver* solver = nullptr;
    if (args.method == "Tabu") {
        solver = new DARPMDTabuSolver(instance, args.time_limit);
    } else {
        solver = new DARPMDSolver(instance, args.time_limit);
    }

    // Solve and Get Results
    solver->solve();
    DARPMD_ResultInstance result = solver->getResult();
    result.displaySummary();
    result.saveToJSON(args.output_path);

    delete solver;
    return 0;
}