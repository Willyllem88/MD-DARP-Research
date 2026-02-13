#pragma once

// Configuration for the ALNS
struct ALNSParams {
    int maxIterations = 2000;
    int setPartitioningInterval = 250; // Run CPLEX SP every X iterations
    double initialTemperature = 100.0;
    double coolingRate = 0.9995;
    double destroyFraction = 0.4; // Fraction of requests to remove in destroy phase
    double worstRemovalPower = 3.0; // For destroyWorst
    
    // Penalties
    double timeWindowPenalty = 100.0;           // per minute
    double vehicleMaxRouteTimePenalty = 100.0;  // per minute
    double capacityPenalty = 100.0;             // per unit
    double rideTimePenalty = 100.0;             // per minute
    double unassignedPenalty = 100000.0; // per request

    // Punction for adaptive operator selection constants
    const double sigma1 = 33.0; // For new best global
    const double sigma2 = 9.0;  // For better than current
    const double sigma3 = 13.0;  // For accepted (but not better)
    const double reactionFactor = 0.1; // How much to adjust weights based on performance
};