#include <iostream>
#include <string>
#include <optional>
#include <memory>

#include "DARPMD_ProblemInstance.h"
#include "Solver.h"
#include "CPLEXSolver.h"
#include "CPLEXSoftSolver.h"
#include "ALNSSolver.h"

const std::string DEFAULT_INSTANCE_PATH = "/home/guillem/TFG-Guillem/data/instances_static/gracia-4R2V.json";


struct Args {
    std::string instance_path = DEFAULT_INSTANCE_PATH;
    std::string output_path = "solution_report.json";
    std::string method = "ILP"; // or "Tabu" or "ALNS" or "ILPSoft"
    int seed = 42;
    bool verbose = false;
    std::optional<double> time_limit;
};

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [-i instance_path] [-t time_limit] [-o output_path] [-m method] [-s seed] [-v]" << std::endl;
    std::cout << "  -i, --instance   Path to problem instance JSON file" << std::endl;
    std::cout << "  -t, --time       Time limit in seconds (optional)" << std::endl;
    std::cout << "  -o, --output     Path to output solution file" << std::endl;
    std::cout << "  -m, --method     Solver method: 'ILP', 'ILPSoft', 'ALNS', 'ALNS_SP', 'ALNS_SC'" << std::endl;
    std::cout << "  -s, --seed       Random seed for reproducibility" << std::endl;
    std::cout << "  -v, --verbose     Enable verbose output" << std::endl;
    std::cout << "  -h, --help       Show this help message" << std::endl;
    std::cout << "Example: " << program_name << " -i ./gracia-4R2V.json -t 300 -o ./solution.json -m ILP -s 42 -v" << std::endl;
}

Args parseArgs(int argc, char** argv) {
    if (argc == 1) {
        std::cout << "No arguments provided. Using default instance: " << DEFAULT_INSTANCE_PATH << std::endl;
        printUsage(argv[0]);
        std::cout << std::endl << "Waiting for 10 seconds before proceeding..." << std::endl;
        sleep(10); // Give user time to read the message    
    }

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
            if (method != "ILP" && method != "ILPSoft" && method != "ALNS" && method != "ALNS_SP" && method != "ALNS_SC") {
                std::cerr << "Unknown method: " << method << ". Use 'ILP', 'ILPSoft', and 'ALNS[_SP/_SC]'." << std::endl;
                exit(1);
            }
            args.method = method;
        }
        else if (a == "-v" || a == "--verbose") {
            args.verbose = true;
        }
        else if ((a == "-s" || a == "--seed") && i + 1 < argc) {
            args.seed = std::stoi(argv[++i]);
        }
        else if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            exit(0);
        }
        else {
            std::cerr << "Unknown argument: " << a << std::endl;
            exit(1);
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
    instance.checkAndFixTriangleInequality(true, args.verbose); // Check and fix triangle inequality if needed
    instance.displayInfo();

    // Select Solver
    std::unique_ptr<Solver> solver;
    if (args.method == "ILP") {
        solver = std::make_unique<CPLEXSolver>(instance, args.time_limit, args.verbose);
    } else if (args.method == "ILPSoft") {
        solver = std::make_unique<CPLEXSoftSolver>(instance, args.time_limit, args.verbose);
    } else if (args.method == "ALNS" || args.method == "ALNS_SP" || args.method == "ALNS_SC") {
        ALNSSolver::HybridMethod hybridMethod = ALNSSolver::HybridMethod::NONE;
        if (args.method == "ALNS_SP") hybridMethod = ALNSSolver::HybridMethod::SET_PARTITIONING;
        else if (args.method == "ALNS_SC") hybridMethod = ALNSSolver::HybridMethod::SET_COVERING;

        solver = std::make_unique<ALNSSolver>(instance, args.time_limit, hybridMethod, args.seed, args.verbose);
    } else {
        std::cerr << "Invalid method selected." << std::endl;
        return 1;
    }

    // Solve and Get Results
    solver->solve();
    DARPMD_ResultInstance result = solver->getResult();
    result.displaySummary();
    result.saveToJSON(args.output_path);

    return 0;
}