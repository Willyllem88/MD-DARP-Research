#pragma once

// Configuration for the ALNS
struct ALNSParams {
    int maxIterations = 2000;
    int setPartitioningInterval = 250; // Run CPLEX SP every X iterations
    double cplexTimeLimit = 10.0; // Time limit for CPLEX in seconds (per SP solve)
    double w = 0.1; // How much worse can a solution be to still be accepted in the first iterations (relative to the initial solution)
    double coolingRate = 0.9995;
    double destroyFraction = 0.4; // Fraction of requests to remove in destroy phase
    
    // Penalties
    double capacityPenalty = 1000.0;             // (alfa) per unit
    double vehicleMaxRouteTimePenalty = 100.0;   // (beta) per time unit
    double timeWindowPenalty = 100.0;            // (gamma) per time unit
    double rideTimePenalty = 100.0;              // (tau) per time unit
    double unassignedPenalty = 100000.0; // per request

    // Similarity weights for Shaw removal
    double shawDistWeight = 9.0;        // (phi) weight for distance in relatedness calculation
    double shawTimeWeight = 3.0;        // (chi) weight for time window similarity in relatedness calculation
    double shawDemandWeight = 1.0;      // (psi) weight for demand similarity in relatedness calculation

    // Power for random selection in destroyWorst and destroyShaw
    double worstRemovalPower = 3.0;

    // Punction for adaptive operator selection constants
    const double sigma1 = 33.0; // For new best global
    const double sigma2 = 9.0;  // For better than current
    const double sigma3 = 13.0;  // For accepted (but not better)
    const double reactionFactor = 0.1; // How much to adjust weights based on performance

    static ALNSParams fromArgs(const std::vector<std::string>& args) {
        ALNSParams p;

        int i = 0;
        p.maxIterations = std::stoi(args[i++]);
        p.coolingRate = std::stod(args[i++]);
        p.destroyFraction = std::stod(args[i++]);
        p.w = std::stod(args[i++]);

        // Could add more parameters here as needed, following the same pattern
        return p;
    }
};