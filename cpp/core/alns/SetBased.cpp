#include "SetBasedSolver.h"

#include <algorithm>
#include <iostream>
#include <string>

struct SetBasedSolver::Model {
    IloModel model;
    IloObjective obj;
    IloRangeArray requestRows;   // (a) covering / partitioning rows
    IloRangeArray vehicleRows;   // (b) each vehicle used exactly once
    IloNumVarArray vars;         // one column per pooled route
    IloNumArray vals;            // solution values (filled on demand)
    IloCplex cplex;
    std::vector<VarInfo> varInfo;                  // column -> (vehicle, route index)
    std::unordered_map<int, int> vehicleOffset;    // vehicle -> first column of its routes

    Model(IloEnv& e, IloInt nRequests, IloInt nVehicles, bool covering)
        : model(e), obj(IloMinimize(e)),
          requestRows(e, nRequests, 1.0, covering ? IloInfinity : 1.0),
          vehicleRows(e, nVehicles, 1.0, 1.0),
          vars(e), vals(e), cplex(e) {
        cplex.setOut(e.getNullStream());      // Silence output
        cplex.setWarning(e.getNullStream());
        cplex.setParam(IloCplex::Param::Threads, 1);  // Single thread usually faster for these subproblems
    }

    ~Model() {
        cplex.end();
        vals.end();
        vars.endElements();
        vars.end();
        requestRows.endElements();
        requestRows.end();
        vehicleRows.endElements();
        vehicleRows.end();
        obj.end();
        model.end();
    }

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
};

static const char* statusName(IloAlgorithm::Status s) {
    switch (s) {
        case IloAlgorithm::Infeasible: return "Infeasible";
        case IloAlgorithm::Optimal:    return "Optimal";
        case IloAlgorithm::Feasible:   return "Feasible (Time Limit)";
        default:                       return "Unknown";
    }
}

// Constructor: pre-calculates the pickup-node -> row mapping (flat vectors for O(1) access)
SetBasedSolver::SetBasedSolver(const MDDARP_ProblemInstance& data,
                               const ALNSParams& params,
                               ALNSEvaluator& evaluator,
                               Logger& logger,
                               SetModelType type)
    : data(data), params(params), evaluator(evaluator), logger(logger),
      type(type), pool(data, params, *this) {
    int maxId = 0;
    for (int i : data.P) maxId = std::max(maxId, i);
    rowOfNode.assign(maxId + 1, -1);
    nodeOfRow.reserve(data.P.size());
    for (int i : data.P) {
        rowOfNode[i] = static_cast<int>(nodeOfRow.size());
        nodeOfRow.push_back(i);
    }
}

// ============================================================================
// Model construction (column generation from the route pool)
// ============================================================================
void SetBasedSolver::buildColumns(Model& m, const RouteMap& routePool, bool integer) {
    m.model.add(m.obj);              // Objective: minimize total cost
    m.model.add(m.requestRows);      // Constraint (a)
    m.model.add(m.vehicleRows);      // Constraint (b)

    size_t totalCols = 0;
    for (auto const& [k, routes] : routePool) totalCols += routes.size();
    m.varInfo.reserve(totalCols);

    const IloNumVar::Type varType = integer ? ILOBOOL : ILOFLOAT;

    // Iterate through the pool and create a column y_rk for each route
    int vehRow = 0;
    for (auto const& [k, routes] : routePool) {
        m.vehicleOffset[k] = static_cast<int>(m.varInfo.size());

        for (size_t rIdx = 0; rIdx < routes.size(); ++rIdx) {
            const ALNSRoute& route = routes[rIdx];

            IloNumColumn col = m.obj(route.totalCost);       // objective coefficient
            col += m.vehicleRows[vehRow](1.0);               // vehicle row
            for (int nodeId : route.sequence) {              // request rows
                const int row = requestRow(nodeId);
                if (row >= 0) col += m.requestRows[row](1.0);
            }

            m.vars.add(IloNumVar(col, 0.0, 1.0, varType));
            m.varInfo.push_back({k, static_cast<int>(rIdx)});
            col.end();  // Release the column object to prevent memory bloat
        }
        ++vehRow;
    }

    m.cplex.extract(m.model);  // Single extraction once the model is complete
}

// ============================================================================
// MIP solve
// ============================================================================
bool SetBasedSolver::solve(ALNSSolution& newSol, std::string& status, double maxTime, std::optional<double> cutoff) {
    const RouteMap& routePool = pool.getRoutes();
    if (routePool.empty()) return false;   // no routes available

    try {
        Model m(env, static_cast<IloInt>(nodeOfRow.size()),
                static_cast<IloInt>(routePool.size()), isCovering());
        buildColumns(m, routePool, /*integer=*/true);

        m.cplex.setParam(IloCplex::Param::TimeLimit, maxTime);
        if (cutoff.has_value())
            m.cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, cutoff.value() + 1e-6);  // small tolerance

        bool solutionFound = m.cplex.solve();
        status = statusName(m.cplex.getStatus());

        if (!solutionFound) {
            logger.log(std::string("  ") + tag() + " CPLEX found no solution. Status: " +
                       std::to_string(static_cast<int>(m.cplex.getStatus())));
            return false;
        }

        // Reconstruct the solution from the selected columns
        m.cplex.getValues(m.vals, m.vars);
        newSol.routes.clear();
        newSol.unassignedRequests.clear();
        for (int i = 0; i < m.vals.getSize(); ++i) {
            if (m.vals[i] > 0.5) {  // binary tolerance
                const VarInfo& info = m.varInfo[i];
                newSol.routes.push_back(routePool.at(info.vehicleId)[info.routeIdx]);
            }
        }

        // Covering may select a request in several routes: disambiguate first
        if (isCovering()) repairSolution(newSol);
        evaluator.evaluateSolution(newSol);

        logger.log(std::string("  ") + tag() + " Total Routes in Pool: " + std::to_string(m.varInfo.size()));
        logger.log(std::string("  ") + tag() + " CPLEX Status: " + statusName(m.cplex.getStatus()));
        logger.log(std::string("  ") + tag() + " CPLEX time: " + std::to_string(m.cplex.getTime()) + " seconds");
        return true;
    } catch (IloException& e) {
        std::cerr << tag() << " CPLEX Exception: " << e << std::endl;
        e.end();
        return false;
    }
}

// ============================================================================
// LP lower bounds
// ============================================================================

// Solves the LP currently extracted in 'm'. On success 'value' holds the objective.
LPStatus SetBasedSolver::solveLP(Model& m, double maxTime, double& value) {
    m.cplex.setParam(IloCplex::Param::TimeLimit, maxTime);
    if (m.cplex.solve() && m.cplex.getStatus() == IloAlgorithm::Optimal) {
        value = m.cplex.getObjValue();
        return LPStatus::Optimal;
    }
    const auto st = m.cplex.getStatus();
    // Costs are >= 0 and vars are in [0,1], so "infeasible or unbounded" means infeasible
    return (st == IloAlgorithm::Infeasible || st == IloAlgorithm::InfeasibleOrUnbounded)
               ? LPStatus::Infeasible : LPStatus::Unknown;
}

LPBoundResult SetBasedSolver::runLP(FixMode mode, int vehicleId, int routeIdx, double maxTime) {
    LPBoundResult res;
    const RouteMap& routePool = pool.getRoutes();
    if (routePool.empty()) return res;

    const double inf = std::numeric_limits<double>::infinity();

    try {
        Model m(env, static_cast<IloInt>(nodeOfRow.size()),
                static_cast<IloInt>(routePool.size()), isCovering());
        buildColumns(m, routePool, /*integer=*/false);   // same model, continuous y in [0,1]

        // 1. Plain LP relaxation
        double value = 0.0;
        res.status = solveLP(m, maxTime, value);
        if (res.status == LPStatus::Infeasible) { res.rootBound = res.bound = inf; return res; }
        if (res.status != LPStatus::Optimal) return res;
        res.rootBound = res.bound = value;
        if (mode == FixMode::None) return res;

        // 2. Choose the route to fix
        int varIdx = -1;
        if (mode == FixMode::Given) {
            auto off = m.vehicleOffset.find(vehicleId);
            auto rts = routePool.find(vehicleId);
            if (off == m.vehicleOffset.end() || rts == routePool.end() ||
                routeIdx < 0 || routeIdx >= static_cast<int>(rts->second.size())) {
                res.status = LPStatus::Unknown;   // invalid route reference
                return res;
            }
            varIdx = off->second + routeIdx;
        } else { // mode == FixMode::Best
            m.cplex.getValues(m.vals, m.vars);
            double best = -1.0;
            for (int i = 0; i < m.vals.getSize(); ++i) {
                if (m.vals[i] > best) { best = m.vals[i]; varIdx = i; }
            }
        }
        res.fixedVehicleId = m.varInfo[varIdx].vehicleId;
        res.fixedRouteIdx = m.varInfo[varIdx].routeIdx;

        // 3. Force the route (y = 1) and re-solve; CPLEX reuses the current basis
        m.vars[varIdx].setLB(1.0);
        res.status = solveLP(m, maxTime, value);
        if (res.status == LPStatus::Optimal)         res.bound = value;
        else if (res.status == LPStatus::Infeasible) res.bound = inf;   // no solution uses this route
        // Unknown: 'bound' keeps the (weaker but valid) root bound
        return res;
    } catch (IloException& e) {
        std::cerr << tag() << " CPLEX Exception (LP): " << e << std::endl;
        e.end();
        res.status = LPStatus::Unknown;
        return res;
    }
}

LPBoundResult SetBasedSolver::computeLPBound(double maxTime) {
    return runLP(FixMode::None, -1, -1, maxTime);
}
LPBoundResult SetBasedSolver::computeLPBoundFixingRoute(const ALNSRoute& route, double maxTime) {
    auto [vehicleId, routeIdx] = findRouteIndex(route);
    return runLP(FixMode::Given, vehicleId, routeIdx, maxTime);
}
LPBoundResult SetBasedSolver::computeLPBoundFixingBestRoute(double maxTime) {
    return runLP(FixMode::Best, -1, -1, maxTime);
}

// Cost of 'route' after removing request 'reqId'. Evaluated through the evaluator on a
// temporary one-route solution.
// ASSUMPTION: evaluateSolution() refreshes route.totalCost from route.sequence. If
// ALNSEvaluator has a route-level evaluation, use it here instead.
double SetBasedSolver::costWithoutRequest(const ALNSRoute& route, int reqId) {
    ALNSSolution tmp;
    tmp.routes.push_back(route);
    removeRequestFromRoute(tmp.routes.front(), reqId);
    evaluator.evaluateSolution(tmp);
    return tmp.routes.front().totalCost;
}

// For each request served by several routes keep the one where its removal would
// save the least (cheapest place to serve it) and drop it from the others.
// Does NOT re-evaluate: the caller must call evaluator.evaluateSolution(sol).
void SetBasedSolver::repairSolution(ALNSSolution& sol) {
    if (sol.routes.empty()) return;

    // Pass 1: which routes cover each request (unique per route)
    std::vector<std::vector<size_t>> coveredBy(nodeOfRow.size());
    std::vector<int> lastRoute(nodeOfRow.size(), -1);
    for (size_t i = 0; i < sol.routes.size(); ++i) {
        for (int nodeId : sol.routes[i].sequence) {
            const int row = requestRow(nodeId);
            if (row < 0 || lastRoute[row] == static_cast<int>(i)) continue;
            lastRoute[row] = static_cast<int>(i);
            coveredBy[row].push_back(i);
        }
    }

    // Pass 2: savings are computed only for duplicated requests
    std::vector<std::pair<size_t, int>> toRemove;   // {routeIdx, reqId}
    for (size_t row = 0; row < coveredBy.size(); ++row) {
        const auto& cand = coveredBy[row];
        if (cand.size() < 2) continue;

        const int reqId = nodeOfRow[row];
        size_t keep = cand.front();
        double bestSavings = std::numeric_limits<double>::infinity();
        for (size_t ri : cand) {
            const ALNSRoute& r = sol.routes[ri];
            const double savings = r.totalCost - costWithoutRequest(r, reqId);
            if (savings < bestSavings) { bestSavings = savings; keep = ri; }
        }
        for (size_t ri : cand)
            if (ri != keep) toRemove.emplace_back(ri, reqId);
    }

    // Apply all removals (decided on the original routes)
    for (const auto& [ri, reqId] : toRemove)
        removeRequestFromRoute(sol.routes[ri], reqId);
}

void SetBasedSolver::removeRequestFromRoute(ALNSRoute& route, int reqId) const {
    const int deliveryId = reqId + data.N_requests;
    auto it = std::remove_if(route.sequence.begin(), route.sequence.end(),
        [reqId, deliveryId](int nodeId) { return nodeId == reqId || nodeId == deliveryId; });
    route.sequence.erase(it, route.sequence.end());
}

std::pair<int, int> SetBasedSolver::findRouteIndex(const ALNSRoute& route) {
    const auto& routes = pool.getRoutes().at(route.vehicleId);

    for (int i = 0; i < static_cast<int>(routes.size()); ++i) {
        if (routes[i].sequence == route.sequence)
            return {route.vehicleId, i};
    }

    std::cout << "[SetBasedSolver] Warning: route not found in the pool for vehicle " << route.vehicleId << std::endl;
    return {-1, -1};
}