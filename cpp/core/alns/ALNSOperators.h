#pragma once

#include "ALNSSolution.h"

#include <random>

// Forward declarations
class ALNSEvaluator;
struct DARPMD_ProblemInstance;
struct ALNSParams;

class ALNSOperators {
public:
    ALNSOperators(const DARPMD_ProblemInstance& instance, 
                  const ALNSParams& parameters, 
                  ALNSEvaluator& evaluator, 
                  std::mt19937& randomEngine);

    // Destroy Operators
    void destroyRandom(ALNSSolution& sol, int q);
    void destroyWorst(ALNSSolution& sol, int q);
    void destroyShaw(ALNSSolution& sol, int q);

    // Repair Operators
    void repairGreedy(ALNSSolution& sol);
    void repairRegret2(ALNSSolution& sol);

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
    std::mt19937& rng;

    // Auxiliary for Shaw removal
    double calculateRelatedness(int i, int j);
};