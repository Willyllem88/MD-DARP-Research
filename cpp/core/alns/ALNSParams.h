#pragma once

// Configuration for the ALNS
struct ALNSParams {
    int maxIterations = 10000;
    int segmentIterations = 100; // How many iterations before we consider a segment "completed" for adaptive operator selection
    int setPartitioningInterval = 2500; // Run CPLEX SP every X iterations
    double cplexTimeLimit = 1200.0; // Time limit for CPLEX in seconds (per SP solve)
    double w = 0.1883; // How much worse can a solution be to still be accepted in the first iterations (relative to the initial solution)
    double coolingRate = 0.9992;
    double destroyFraction = 0.2466; // Fraction of requests to remove in destroy phase
    
    // Penalties
    double capacityPenalty = 1000.0;             // (alfa) per unit
    double vehicleMaxRouteTimePenalty = 100.0;   // (beta) per time unit
    double timeWindowPenalty = 100.0;            // (gamma) per time unit
    double rideTimePenalty = 100.0;              // (tau) per time unit
    double unassignedPenalty = 10000.0; // per request

    // Similarity weights for Shaw removal
    double shawDistWeight = 10.5576;        // (phi) weight for distance in relatedness calculation
    double shawTimeWeight = 6.4498;        // (chi) weight for time window similarity in relatedness calculation
    double shawDemandWeight = 1.0071;      // (psi) weight for demand similarity in relatedness calculation

    // Power for random selection in destroyWorst and destroyShaw
    double worstRemovalPower = 3.0;

    // Punctuation for adaptive operator selection constants
    double sigma1 = 51.2130; // For new best global
    double sigma2 = 11.1318;  // For better than current
    double sigma3 = 3.7313;  // For accepted (but not better)
    double reactionFactor = 0.1; // How much to adjust weights based on performance

    // TODO: add an enum to decide if use exact evaluation o greedy insertion cost

    static ALNSParams fromArgs(const std::vector<std::string>& args) {
        ALNSParams p;

        int i = 0;
        p.maxIterations = std::stoi(args[i++]);
        p.w = std::stod(args[i++]);
        p.coolingRate = std::stod(args[i++]);
        p.destroyFraction = std::stod(args[i++]);

        p.shawDistWeight = std::stod(args[i++]);
        p.shawTimeWeight = std::stod(args[i++]);
        p.shawDemandWeight = std::stod(args[i++]);

        p.sigma1 = std::stod(args[i++]);
        p.sigma2 = std::stod(args[i++]);
        p.sigma3 = std::stod(args[i++]);

        // Could add more parameters here as needed, following the same pattern
        return p;
    }
};