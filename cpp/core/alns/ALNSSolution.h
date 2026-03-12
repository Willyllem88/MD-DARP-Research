#pragma once

#include <vector> 
#include <set>

#include "ALNSRoute.h"

// Internal representation of a full solution
struct ALNSSolution {
    std::vector<ALNSRoute> routes; // One per vehicle
    std::set<int> unassignedRequests; // IDs of requests in P not served
    double objectiveValue = 0.0; // Total penalized cost
};