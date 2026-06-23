#pragma once

#include "../MDDARP_ProblemInstance.h"
#include "../logger.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"
#include "RoutePool.h"

#include <ilcplex/ilocplex.h>

class SetBasedSolver {
public:
    SetBasedSolver(
        const MDDARP_ProblemInstance& data, 
        const ALNSParams& params, 
        ALNSEvaluator& evaluator,
        Logger& logger
    ): data(data), params(params), evaluator(evaluator), logger(logger), pool(data) { }

    virtual ~SetBasedSolver() {
        env.end();
    }

    // Solve the Set Partitioning/Covering problem using the accumulated 
    // routePool and return the best solution found, returns true if a solution was found, false otherwise
    virtual bool solve(ALNSSolution& newSol, double maxTime = 60.0) = 0;

    // Accessor for the route pool, so the ALNSSolver can add routes to it 
    // during the ALNS search
    RoutePool& getRoutePool() { return pool; }

protected:
    const MDDARP_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
    Logger& logger;
    
    // CPLEX environment
    IloEnv env;

    // Pool of routes generated during the ALNS search
    RoutePool pool;
};