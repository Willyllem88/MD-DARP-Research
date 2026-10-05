//INFO: for ALNS
#pragma once

#include "../MDDARP_ProblemInstance.h"
#include "../logger.h"
#include "ALNSSolution.h"
#include "ALNSParams.h"
#include "ALNSEvaluator.h"
#include "RoutePool.h"

#include <ilcplex/ilocplex.h>

#include <limits>
#include <unordered_map>
#include <vector>

// Which master model the solver builds.
//  PARTITIONING: every request covered exactly once
//  COVERING:     every request covered at least once,
//                duplicates are removed afterwards by repairSolution()
enum class SetModelType { PARTITIONING, COVERING };

enum class LPStatus { Optimal, Infeasible, Unknown };

// Result of an LP-based lower bound computation.
//  rootBound: optimal value of the plain LP relaxation of the model in use.
//  bound:     value after fixing a route to 1 (== rootBound if nothing was
//             fixed). It is a lower bound only on solutions that contain that route.
//  Values are always safe: -inf = nothing known, +inf = proven infeasible.
struct LPBoundResult {
    LPStatus status = LPStatus::Unknown;
    double rootBound = -std::numeric_limits<double>::infinity();
    double bound = -std::numeric_limits<double>::infinity();
    int fixedVehicleId = -1;   // route that was fixed (-1 if none)
    int fixedRouteIdx = -1;    // index inside the route pool of that vehicle
};

class SetBasedSolver {
public:
    SetBasedSolver(
        const MDDARP_ProblemInstance& data,
        const ALNSParams& params,
        ALNSEvaluator& evaluator,
        Logger& logger,
        SetModelType type
    );

    // The persistent CPLEX environment is owned by this object.
    ~SetBasedSolver() { env.end(); }
    SetBasedSolver(const SetBasedSolver&) = delete;
    SetBasedSolver& operator=(const SetBasedSolver&) = delete;

    // Solve the MIP over the accumulated route pool. Returns true if a solution
    // was found. 'cutoff' (optional) prunes solutions costlier than that value.
    bool solve(ALNSSolution& newSol, double maxTime = 60.0,
               double cutoff = std::numeric_limits<double>::infinity());

    // First lower bound: LP relaxation of the model selected by SetModelType.
    LPBoundResult computeLPBound(double maxTime = 60.0);

    // Second lower bound: same LP with route forced to be used.
    LPBoundResult computeLPBoundFixingRoute(const ALNSRoute& route, double maxTime = 60.0);

    // Second lower bound: solves the LP, fixes the route with the largest LP value
    // to 1 and re-solves (warm started). Chosen route is reported in the result.
    LPBoundResult computeLPBoundFixingBestRoute(double maxTime = 60.0);

    // Accessor so the ALNS can add routes to the pool during the search.
    RoutePool& getRoutePool() { return pool; }
    SetModelType getType() const { return type; }

private:
    // Must match the type returned by RoutePool::getRoutes().
    using RouteMap = std::unordered_map<int, std::vector<ALNSRoute>>;

    struct VarInfo { int vehicleId; int routeIdx; };
    struct Model;  // RAII bundle of Concert objects, defined in the .cpp
    enum class FixMode { None, Given, Best };

    const MDDARP_ProblemInstance& data;
    const ALNSParams& params;
    ALNSEvaluator& evaluator;
    Logger& logger;
    const SetModelType type;

    IloEnv env;
    RoutePool pool;

    std::vector<int> rowOfNode;  // node id -> request row (-1 if not a pickup)
    std::vector<int> nodeOfRow;  // request row -> pickup node id

    bool isCovering() const { return type == SetModelType::COVERING; }
    const char* tag() const { return isCovering() ? "[SetCovering]" : "[SetPartitioning]"; }
    int requestRow(int nodeId) const {
        return (nodeId >= 0 && static_cast<size_t>(nodeId) < rowOfNode.size()) ? rowOfNode[nodeId] : -1;
    }

    // Model construction / LP helpers
    void buildColumns(Model& m, const RouteMap& routePool, bool integer);
    LPStatus solveLP(Model& m, double maxTime, double& value);
    LPBoundResult runLP(FixMode mode, int vehicleId, int routeIdx, double maxTime);

    // Covering repair (duplicates removal)
    void repairSolution(ALNSSolution& sol);
    double costWithoutRequest(const ALNSRoute& route, int reqId);
    void removeRequestFromRoute(ALNSRoute& route, int reqId) const;

    // Find the index of a route in the route pool, returning the vehicle ID and route index.
    std::pair<int, int> findRouteIndex(const ALNSRoute& route);
};