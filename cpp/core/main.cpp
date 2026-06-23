#include <iostream>
#include <string>
#include <optional>
#include <memory>

#include "MDDARP_ProblemInstance.h"
#include "Solver.h"
#include "CPLEXSolver.h"
#include "CPLEXSoftSolver.h"
#include "ALNSSolver.h"

const std::string DEFAULT_INSTANCE_PATH = "/home/guillem/TFG-Guillem/data/gracia-4R2V.json";


struct Args {
    std::string instance_path = DEFAULT_INSTANCE_PATH;
    std::string output_path = "solution_report.json";
    std::string method = "ILP"; // or "ILPSoft" or "ALNS" or "ALNS_SP" or "ALNS_SC"
    int seed = 42;
    bool verbose = false;
    std::optional<double> time_limit;
    std::vector<std::string> alnsParams;
    bool enableGICE = false;
    bool enableNR = false;
};

void printArgsSummary(const Args& args) {
    std::cout << "=== Arguments Summary === " << std::endl;
    std::cout << "  Instance Path: " << args.instance_path << std::endl;
    std::cout << "  Output Path: " << args.output_path << std::endl;
    std::cout << "  Method: " << args.method << std::endl;
    std::cout << "  Seed: " << args.seed << std::endl;
    std::cout << "  Verbose: " << (args.verbose ? "Yes" : "No") << std::endl;
    if (args.time_limit.has_value()) {
        std::cout << "  Time Limit: " << args.time_limit.value() << " seconds" << std::endl;
    } else {
        std::cout << "  Time Limit: None" << std::endl;
    }
    if (!args.alnsParams.empty()) {
        std::cout << "  ALNS Params: ";
        for (const auto& param : args.alnsParams) {
            std::cout << param << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "  Enable GICE: " << (args.enableGICE ? "Yes" : "No") << std::endl;
    std::cout << "  Enable NR: " << (args.enableNR ? "Yes" : "No") << std::endl;
    std::cout << "=========================" << std::endl << std::endl;
}

void printUsage(const char* program_name) {
    std::cout << "MD-DARP Solver - Usage Instructions\n"
              << "===================================\n"
              << "This program solves the Multi-Depot Dial-a-Ride Problem using various methods.\n\n"
              << "Usage: " << program_name << R"( [-i instance_path] [-t time_limit] [-o output_path]
      [-m method] [-s seed] [-v] [--GICE] [--NR]
      [--alnsParams maxIterations w coolingRate destroyFraction
                    shawDistWeight shawTimeWeight shawDemandWeight
                    sigma1 sigma2 sigma3]

  -i, --instance   Path to problem instance JSON file
  -t, --time       Time limit in seconds
  -o, --output     Path to output solution file
  -m, --method     Solver method: ILP, ILPSoft, ALNS, ALNS_SP, ALNS_SC
  -s, --seed       Random seed for reproducibility
  -v, --verbose    Enable verbose output
  --alnsParams     Additional ALNS parameters in order (maxIterations, w, coolingRate,
                    destroyFraction, shawDistWeight, shawTimeWeight, shawDemandWeight,
                    sigma1, sigma2, sigma3)
  --GICE           Enable Greedy Insertion Cost Evaluator in ALNS, if not set uses Forward
                    Time Slack Evaluator (FTSE)
  --NR             Enable Neighbor Reduction in ALNS
  -h, --help       Show this help message
)"
              << "\nExample: " << program_name
              << " -i ./a2-16.json -t 300 -o ./solution.json -m ILP -s 42 -v\n";
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
        }
        else if ((a == "-t" || a == "--time") && i + 1 < argc) {
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
        else if (a == "--alnsParams") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                args.alnsParams.push_back(argv[++i]);
            }
        }
        else if (a == "--GICE") {
            args.enableGICE = true;
        }
        else if (a == "--NR") {
            args.enableNR = true;
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
    if (args.verbose) printArgsSummary(args);

    // Load Problem Instance
    MDDARP_ProblemInstance instance;
    if (!instance.loadFromJSON(args.instance_path)) {
        std::cerr << "Failed to load instance from " << args.instance_path << std::endl;
        return 1;
    }
    instance.checkAndFixTriangleInequality(true, args.verbose); // Check and fix triangle inequality if needed
    if(args.verbose) instance.displayInfo();

    // Select Solver
    std::unique_ptr<Solver> solver;
    if (args.method == "ILP") {
        solver = std::make_unique<CPLEXSolver>(instance, args.time_limit, args.verbose);

    } else if (args.method == "ILPSoft") {
        solver = std::make_unique<CPLEXSoftSolver>(instance, args.time_limit, args.verbose);

    } else if (args.method == "ALNS" || args.method == "ALNS_SP" || args.method == "ALNS_SC") {
        // Hybrid method selection
        ALNSSolver::HybridMethod hybridMethod = ALNSSolver::HybridMethod::NONE;
        if (args.method == "ALNS_SP") hybridMethod = ALNSSolver::HybridMethod::SET_PARTITIONING;
        else if (args.method == "ALNS_SC") hybridMethod = ALNSSolver::HybridMethod::SET_COVERING;

        // Parse ALNS parameters from command line or use defaults
        ALNSParams params = !args.alnsParams.empty() 
            ? ALNSParams::fromArgs(args.alnsParams)
            : ALNSParams();

        solver = std::make_unique<ALNSSolver>(
            instance,
            args.time_limit,
            hybridMethod, args.seed,
            args.verbose,
            params,
            args.enableGICE,
            args.enableNR
        );
    } else {
        std::cerr << "Invalid method selected." << std::endl;
        return 1;
    }

    // Solve and Get Results
    solver->solve();
    MDDARP_ResultInstance result = solver->getResult();
    if(args.verbose) result.displaySummary();
    result.saveToJSON(args.output_path);
    if(args.verbose) std::cout << "Solution saved to: " << args.output_path << std::endl;
    std::cout << result.objectiveValue << std::endl;

    return 0;
}