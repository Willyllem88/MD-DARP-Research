#pragma once

#include "ALNSSolution.h"

#include <random>

// Forward declarations
class ALNSEvaluator;
struct MDDARP_ProblemInstance;
struct ALNSParams;

class ALNSOperators {
public:
    ALNSOperators(const MDDARP_ProblemInstance& instance, 
                  const ALNSParams& parameters, 
                  ALNSEvaluator& evaluator, 
                  std::mt19937& randomEngine,
                  bool enableNR = false);

    // Destroy Operators
    void destroyRandom(ALNSSolution& sol, int q);
    void destroyWorst(ALNSSolution& sol, int q);
    void destroyShaw(ALNSSolution& sol, int q);

    // Repair Operators
    void repairGreedy(ALNSSolution& sol);
    void repairRegret2(ALNSSolution& sol);
    void repairRegret3(ALNSSolution& sol);

private:
    const MDDARP_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
    std::mt19937& rng;

    // Auxiliary for Shaw removal
    double calculateRelatedness(int i, int j, const ALNSSolution& sol);

    // Insertion evaluation methods
    struct LocalInsertion {
        int pIdx = -1;
        int dIdx = -1;
        double deltaCost = std::numeric_limits<double>::max();
    };
    enum ReductionMethod { REDUCTION, NONE };
    ReductionMethod reductionMethod = NONE;
    LocalInsertion findBestInsertion(const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionExact(const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionExact_R(const ALNSRoute& route, int reqId);

    // Cache for regret-2 and regret-3 insertions to avoid redundant calculations
    std::vector<std::vector<LocalInsertion>> insertionCache;
     
};