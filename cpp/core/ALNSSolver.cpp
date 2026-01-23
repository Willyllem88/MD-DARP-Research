#include "ALNSSolver.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

ALNSSolver::ALNSSolver(const DARPMD_ProblemInstance& instance, std::optional<double> timeLimit)
    : data(instance), timeLimit(timeLimit) {
    
    // Seed random number generator
    std::random_device rd;
    rng = std::mt19937(rd());
}

ALNSSolver::~ALNSSolver() {
    // Nothing special to clean up unless we keep persistent CPLEX envs
}

// ------------------------------------------------------------------
// Core Logic: Route Evaluation (Handles Time Windows & Penalties)
// ------------------------------------------------------------------
void ALNSSolver::evaluateRoute(ALNSRoute& route) {
    route.distanceCost = 0.0;
    route.timeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;
    route.arrivalTimes.clear();
    route.loads.clear();

    if (route.sequence.empty()) return;

    double currentTime = 0.0; // Start time is usually 0 or earliest window
    double currentLoad = 0.0;
    
    // 1. Initialize at Start Depot
    int startNode = route.sequence.front();
    double startWindow = data.getTimeWindowStart(startNode);
    currentTime = std::max(0.0, startWindow); 
    
    route.arrivalTimes[startNode] = currentTime;
    route.loads[startNode] = 0.0;

    // Tracking for Ride Time constraints: key = request_id (pickup node), val = pickup_time
    std::map<int, double> pickupTimes;

    for (size_t i = 0; i < route.sequence.size() - 1; ++i) {
        int u = route.sequence[i];
        int v = route.sequence[i+1];

        // --- Distance ---
        // Note: Assuming vehicleId matches index in capacity/time vectors
        route.distanceCost += data.getCost(u, v, route.vehicleId);

        // --- Time ---
        double serviceTime = data.getServiceTime(u);
        double travelTime = data.getTravelTime(u, v);
        
        double arrivalAtV = currentTime + serviceTime + travelTime;
        
        // Time Window Check at V
        double earlyTW = data.getTimeWindowStart(v);
        double lateTW = data.getTimeWindowEnd(v);

        // If early, we wait (Time warp is not a violation usually, but waiting)
        if (arrivalAtV < earlyTW) {
            arrivalAtV = earlyTW;
        }
        
        // If late, penalty
        if (arrivalAtV > lateTW) {
            route.timeViolation += (arrivalAtV - lateTW);
            // In a soft TW context, we assume we arrive 'late' but continue
            // To prevent massive propagation, sometimes we clamp, but here we let it flow
        }

        currentTime = arrivalAtV;
        route.arrivalTimes[v] = currentTime;

        // --- Capacity ---
        double demand = data.getDemand(v);
        currentLoad += demand;
        route.loads[v] = currentLoad;

        double capacity = data.capacity.at(route.vehicleId);
        if (currentLoad > capacity) {
            route.loadViolation += (currentLoad - capacity);
        }

        // --- Ride Time Logic ---
        // If v is a pickup node (in P)
        if (std::find(data.P.begin(), data.P.end(), v) != data.P.end()) {
            pickupTimes[v] = currentTime;
        }
        // If v is a delivery node (in D)
        else if (std::find(data.D.begin(), data.D.end(), v) != data.D.end()) {
            // Find corresponding pickup ID. Assuming D_id = P_id + N_requests
            int pickupId = v - data.N_requests; 
            if (pickupTimes.count(pickupId)) {
                double rideTime = currentTime - (pickupTimes[pickupId] + data.getServiceTime(pickupId));
                if (rideTime > data.max_ride_time) {
                    route.rideTimeViolation += (rideTime - data.max_ride_time);
                }
            }
        }
    }

    // Check Max Route Duration
    int endNode = route.sequence.back();
    double duration = route.arrivalTimes[endNode] - route.arrivalTimes[startNode];
    if (duration > data.max_route_time.at(route.vehicleId)) {
        route.timeViolation += (duration - data.max_route_time.at(route.vehicleId));
    }

    // Final Cost Aggregation
    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeViolation == 0 && 
                        route.loadViolation == 0 && 
                        route.rideTimeViolation == 0);
}

void ALNSSolver::evaluateSolution(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRoute(r);
        sol.objectiveValue += r.totalCost;
        
        // Add valid routes to pool for later Set Partitioning
        if (r.isFeasible && r.sequence.size() > 2) {
            addRouteToPool(r);
        }
    }
    // Penalize unassigned
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

// ------------------------------------------------------------------
// Set Partitioning using CPLEX
// ------------------------------------------------------------------
void ALNSSolver::addRouteToPool(const ALNSRoute& route) {
    // Simple deduplication check could be added here
    routePool[route.vehicleId].push_back(route);
}

void ALNSSolver::solveSetPartitioning() {
    // This method takes all routes in routePool and solves an SP model
    if (routePool.empty()) return;

    std::cout << "  [Matheuristic] Running Set Partitioning on Pool..." << std::endl;

    IloEnv env;
    try {
        IloModel model(env);
        IloCplex spCplex(model);
        spCplex.setOut(env.getNullStream()); // Silence output

        // 1. Variables: y_rk = 1 if route r of vehicle k is selected
        std::map<std::pair<int, int>, IloNumVar> y; // <vehicle, route_idx>
        
        // z_i = 1 if request i is unassigned (Slack variable)
        std::map<int, IloNumVar> z; 

        IloExpr objExpr(env);

        // Add Route Variables
        for (auto const& [k, routes] : routePool) {
            for (size_t rIdx = 0; rIdx < routes.size(); ++rIdx) {
                // Name helps debugging
                std::string name = "y_" + std::to_string(k) + "_" + std::to_string(rIdx);
                y[{k, rIdx}] = IloNumVar(env, 0, 1, ILOBOOL, name.c_str());
                
                // Objective: minimize route cost
                objExpr += routes[rIdx].distanceCost * y[{k, rIdx}];
            }
        }

        // Add Slack Variables (Unassigned)
        for (int i : data.P) {
            z[i] = IloNumVar(env, 0, 1, ILOBOOL);
            objExpr += params.unassignedPenalty * z[i];
        }

        model.add(IloMinimize(env, objExpr));

        // 2. Constraints
        
        // (a) Each request covered exactly once (or dropped via slack)
        for (int i : data.P) {
            IloExpr coverExpr(env);
            for (auto const& [k, routes] : routePool) {
                for (size_t rIdx = 0; rIdx < routes.size(); ++rIdx) {
                    // Check if request i is in this route
                    const auto& seq = routes[rIdx].sequence;
                    if (std::find(seq.begin(), seq.end(), i) != seq.end()) {
                        coverExpr += y[{k, rIdx}];
                    }
                }
            }
            coverExpr += z[i];
            model.add(coverExpr == 1);
            coverExpr.end();
        }

        // (b) Each vehicle used at most once
        for (int k : data.K) {
            if (routePool.find(k) == routePool.end()) continue;
            IloExpr vehicleUsage(env);
            for (size_t rIdx = 0; rIdx < routePool[k].size(); ++rIdx) {
                vehicleUsage += y[{k, rIdx}];
            }
            model.add(vehicleUsage <= 1);
            vehicleUsage.end();
        }

        // 3. Solve
        spCplex.setParam(IloCplex::Param::TimeLimit, 10.0); // Strict limit for SP
        if (spCplex.solve()) {
            // Reconstruct Solution
            ALNSSolution newSol;
            newSol.unassignedRequests.clear();

            // Active Routes
            for (auto const& [key, var] : y) {
                if (spCplex.getValue(var) > 0.5) {
                    int k = key.first;
                    int rIdx = key.second;
                    newSol.routes.push_back(routePool[k][rIdx]);
                }
            }
            // Fill empty routes for unused vehicles
            std::set<int> usedVehicles;
            for(auto& r : newSol.routes) usedVehicles.insert(r.vehicleId);
            for(int k : data.K) {
                if(usedVehicles.find(k) == usedVehicles.end()) {
                    ALNSRoute emptyR;
                    emptyR.vehicleId = k;
                    emptyR.sequence = {data.StartNode.at(k), data.EndNode.at(k)};
                    evaluateRoute(emptyR);
                    newSol.routes.push_back(emptyR);
                }
            }

            // Unassigned
            for (int i : data.P) {
                if (spCplex.getValue(z[i]) > 0.5) {
                    newSol.unassignedRequests.insert(i);
                }
            }

            evaluateSolution(newSol);
            
            // If CPLEX found something better, update global best
            if (newSol.objectiveValue < bestObjective) {
                std::cout << "  [Matheuristic] CPLEX found new best: " << newSol.objectiveValue << std::endl;
                bestSolution = newSol;
                bestObjective = newSol.objectiveValue;
            }
        }

    } catch (IloException& e) {
        std::cerr << "CPLEX Exception in SP: " << e << std::endl;
    }
    env.end();
}

// ------------------------------------------------------------------
// High-Level ALNS Methods
// ------------------------------------------------------------------

ALNSSolution ALNSSolver::createInitialSolution() {
    ALNSSolution sol;
    
    // Initialize empty routes
    for (int k : data.K) {
        ALNSRoute r;
        r.vehicleId = k;
        r.sequence.push_back(data.StartNode.at(k));
        r.sequence.push_back(data.EndNode.at(k));
        evaluateRoute(r); // Zero cost initially
        sol.routes.push_back(r);
    }

    // All requests start as unassigned
    for (int i : data.P) {
        sol.unassignedRequests.insert(i);
    }
    
    // Apply greedy repair to insert as many as possible
    repairGreedy(sol);
    evaluateSolution(sol);
    
    return sol;
}

void ALNSSolver::destroyRandom(ALNSSolution& sol, int q) {
    // Randomly remove q requests
    std::vector<int> requestsInRoutes;
    // Identify where everyone is
    struct Loc { int vIdx; int nodeIdx; int reqId; };
    std::vector<Loc> locations;

    for (size_t v = 0; v < sol.routes.size(); ++v) {
        const auto& seq = sol.routes[v].sequence;
        for (size_t i = 1; i < seq.size() - 1; ++i) {
            // If it's a pickup node
            if (std::find(data.P.begin(), data.P.end(), seq[i]) != data.P.end()) {
                locations.push_back({(int)v, (int)i, seq[i]});
            }
        }
    }

    std::shuffle(locations.begin(), locations.end(), rng);
    
    int removed = 0;
    for (const auto& loc : locations) {
        if (removed >= q) break;

        // Remove pickup (loc.reqId) and delivery (reqId + N) from route
        auto& route = sol.routes[loc.vIdx];
        
        // Simple removal: iterate backwards to keep indices safe roughly, 
        // or just rebuild vector
        std::vector<int> newSeq;
        int deliveryId = loc.reqId + data.N_requests;
        
        for (int node : route.sequence) {
            if (node != loc.reqId && node != deliveryId) {
                newSeq.push_back(node);
            }
        }
        route.sequence = newSeq;
        
        sol.unassignedRequests.insert(loc.reqId);
        removed++;
    }
}

void ALNSSolver::repairGreedy(ALNSSolution& sol) {
    // Try to insert unassigned requests into best positions
    // Simple version: iterate unassigned, find best spot, insert, repeat
    
    std::vector<int> todo(sol.unassignedRequests.begin(), sol.unassignedRequests.end());
    sol.unassignedRequests.clear(); // Will re-add failures later

    // Shuffle to vary the greedy order
    std::shuffle(todo.begin(), todo.end(), rng);

    for (int reqId : todo) {
        int deliveryId = reqId + data.N_requests;
        
        double bestCostIncrease = std::numeric_limits<double>::max();
        int bestVehicle = -1;
        int bestPIdx = -1;
        int bestDIdx = -1;

        // Try all vehicles
        for (size_t v = 0; v < sol.routes.size(); ++v) {
            const auto& seq = sol.routes[v].sequence;
            
            // Try all insertion pairs (i, j) where i < j
            // sequence: 0 ... i ... j ... end
            for (size_t i = 1; i < seq.size(); ++i) {
                for (size_t j = i; j < seq.size(); ++j) {
                    
                    // Construct temp route
                    ALNSRoute temp = sol.routes[v];
                    temp.sequence.insert(temp.sequence.begin() + i, reqId);
                    temp.sequence.insert(temp.sequence.begin() + j + 1, deliveryId);
                    
                    evaluateRoute(temp);
                    
                    double increase = temp.totalCost - sol.routes[v].totalCost;
                    
                    if (increase < bestCostIncrease) {
                        bestCostIncrease = increase;
                        bestVehicle = v;
                        bestPIdx = i;
                        bestDIdx = j + 1;
                    }
                }
            }
        }

        // Apply best move if reasonable
        if (bestVehicle != -1 && bestCostIncrease < params.unassignedPenalty) {
            auto& r = sol.routes[bestVehicle];
            r.sequence.insert(r.sequence.begin() + bestPIdx, reqId);
            r.sequence.insert(r.sequence.begin() + bestDIdx, deliveryId);
            // Don't need to eval full solution yet, loop handles local delta
            // But we should update the route cost locally for next comparison
            evaluateRoute(r);
        } else {
            sol.unassignedRequests.insert(reqId);
        }
    }
}

// ------------------------------------------------------------------
// Main Solve Loop
// ------------------------------------------------------------------
void ALNSSolver::solve() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Starting ALNS search..." << std::endl;

    // 1. Initial Solution
    ALNSSolution currentSol = createInitialSolution();
    bestSolution = currentSol;
    bestObjective = currentSol.objectiveValue;

    std::cout << "Initial Objective: " << bestObjective << std::endl;

    double temperature = params.initialTemperature;
    
    // 2. Main Loop
    for (int iter = 0; iter < params.maxIterations; ++iter) {
        
        // Check time limit
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (timeLimit.has_value() && elapsed.count() > timeLimit.value()) {
            std::cout << "Time limit reached." << std::endl;
            break;
        }

        // Copy current
        ALNSSolution neighbor = currentSol;

        // --- Destroy ---
        // Adaptive logic would go here. For now, random.
        // Remove ~20% of requests or min 2
        int q = std::max(2, (int)(data.P.size() * 0.2));
        destroyRandom(neighbor, q);

        // --- Repair ---
        repairGreedy(neighbor);
        evaluateSolution(neighbor);

        // --- Acceptance (Simulated Annealing) ---
        double delta = neighbor.objectiveValue - currentSol.objectiveValue;
        
        bool accept = false;
        if (delta < 0) {
            accept = true;
        } else {
            double prob = std::exp(-delta / temperature);
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(rng) < prob) {
                accept = true;
            }
        }

        if (accept) {
            currentSol = neighbor;
            if (currentSol.objectiveValue < bestObjective) {
                bestSolution = currentSol;
                bestObjective = currentSol.objectiveValue;
                std::cout << "Iter " << iter << ": New Best = " << bestObjective 
                          << " (Violations: " << (bestSolution.objectiveValue > 10000 ? "Yes" : "No") << ")" << std::endl;
            }
        }

        // Cooling
        temperature *= params.coolingRate;

        // --- Matheuristic Integration ---
        if (iter > 0 && iter % params.setPartitioningInterval == 0) {
            solveSetPartitioning();
            // Optional: Re-inject the CPLEX solution as the current solution for ALNS
            // currentSol = bestSolution; 
        }
    }

    // Final clean run of SP
    solveSetPartitioning();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = end - start;
    this->solveTime = totalElapsed.count();
}


// ------------------------------------------------------------------
// Result Formatting
// ------------------------------------------------------------------
DARPMD_ResultInstance ALNSSolver::getResult() const {
    DARPMD_ResultInstance result(data);
    result.solveTime = this->solveTime;
    result.objectiveValue = this->bestObjective;
    
    // If we have violations, mark as Feasible only (or Infeasible)
    // But since this is heuristic result, we return what we have.
    result.solverStatus = (bestSolution.unassignedRequests.empty()) ? "Feasible" : "Partial/Infeasible";

    for (const auto& r : bestSolution.routes) {
        VehicleRoute vRoute;
        vRoute.vehicleId = r.vehicleId;

        for (int nodeId : r.sequence) {
            RouteStep step;
            step.nodeId = nodeId;
            
            // Types
            if (nodeId == data.StartNode.at(r.vehicleId)) step.type = "DepotStart";
            else if (nodeId == data.EndNode.at(r.vehicleId)) step.type = "DepotEnd";
            else if (std::find(data.P.begin(), data.P.end(), nodeId) != data.P.end()) step.type = "Pickup";
            else step.type = "Delivery";

            // Timing info from evaluation
            if (r.arrivalTimes.count(nodeId)) step.arrivalTime = r.arrivalTimes.at(nodeId);
            else step.arrivalTime = 0.0;

            if (r.loads.count(nodeId)) step.loadAfter = r.loads.at(nodeId);
            else step.loadAfter = 0.0;

            vRoute.steps.push_back(step);
        }
        result.addRoute(r.vehicleId, vRoute);
    }

    return result;
}

//TODO: more opeators