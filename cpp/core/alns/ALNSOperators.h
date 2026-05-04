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
                  std::mt19937& randomEngine,
                  bool enableGICE = false,
                  bool enableNR = false);

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

    // Insertion evaluation methods
    struct LocalInsertion {
        int pIdx = -1;
        int dIdx = -1;
        double deltaCost = std::numeric_limits<double>::max();
    };
    enum InsertionMethod { FTSE, GICE }; // Forward Time Slack Evaluation, Greedy Insertion Cost Evaluation
    enum ReductionMethod { REDUCTION, NONE };
    InsertionMethod insertionMethod = FTSE;
    ReductionMethod reductionMethod = NONE;
    LocalInsertion findBestInsertion(InsertionMethod method, const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionExact(const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionGreedy(const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionExact_R(const ALNSRoute& route, int reqId);
    LocalInsertion findBestInsertionGreedy_R(const ALNSRoute& route, int reqId);
     
};