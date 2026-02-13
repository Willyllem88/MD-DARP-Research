#pragma once

#include "../DARPMD_ProblemInstance.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"

#include <map>

class SetPartitioningSolver {
public:
    SetPartitioningSolver(
        const DARPMD_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator
    );

    ALNSSolution solve(const std::map<int, std::vector<ALNSRoute>>& routePool) const;

private:
    const DARPMD_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
};
