#pragma once

#include <vector>
#include <string>
#include <chrono>

struct ALNSIterationRecord {
    int iteration;
    double elapsedTime;
    double currentObjective;
    double bestObjective;
    double bestFeasibleObjective;
    int destroyOpIdx;
    int repairOpIdx;
    double reward;
    double temperature;
    int segment;
};

struct ALNSWeightsRecord {
    int segment;
    double elapsedTime;
    std::vector<double> destroyWeights;
    std::vector<double> repairWeights;
};

class ALNSLogger {
public:
    ALNSLogger(size_t expectedIterations = 10000, size_t expectedSegments = 100);

    void recordIteration(int iter, double time, double currObj, double bestObj, 
                         double bestFeasibleObj, int destOp, int repOp, 
                         double reward, double temp, int segment);

    void recordWeights(int segment, double time, 
                       const std::vector<double>& destWeights, 
                       const std::vector<double>& repWeights);

    void exportConvergenceCSV(const std::string& filepath) const;
    
    void exportWeightsEvolutionCSV(const std::string& filepath) const;

private:
    std::vector<ALNSIterationRecord> iterationHistory;
    std::vector<ALNSWeightsRecord> weightsHistory;
};