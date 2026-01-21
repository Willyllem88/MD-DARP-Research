#include "DARPMDTabuSolver.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>

DARPMDTabuSolver::DARPMDTabuSolver(const DARPMD_ProblemInstance& instance, std::optional<double> timeLimit)
    : data(instance), timeLimit(timeLimit) {
    
    // Initialize Random Seed
    rng.seed(42);

    // Resize matrices
    // +1 for safety on indices if requests are 1-based, assuming 0..N_requests-1 here for vector indexing
    int numRequests = data.P.size(); 
    int numVehicles = data.K.size();
    
    // Map max ID to size vectors
    int maxReqId = 0;
    for(int p : data.P) maxReqId = std::max(maxReqId, p);
    
    tabuList.resize(maxReqId + 1, std::vector<int>(numVehicles + 1, 0));
    freqMatrix.resize(maxReqId + 1, std::vector<int>(numVehicles + 1, 0));
}

// ---------------------------------------------------------
// Main Solve Loop
// ---------------------------------------------------------
void DARPMDTabuSolver::solve() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Starting Tabu Search..." << std::endl;

    // 1. Initial Solution [Section 4.4]
    TabuSolution currentSol = generateInitialSolution();
    evaluateSolution(currentSol);
    
    bestSol = currentSol; // Best visited (even if infeasible)
    
    if (currentSol.isFeasible()) {
        bestFeasibleSolution = currentSol;
        feasibleFound = true;
    }

    const double MIN_PENALTY = 0.1;
    const double MAX_PENALTY = 10000.0;
    const double delta = 0.5;

    int iter = 0;
     // Update factor [Section 4.5]

    while (iter < maxIterations) {
        std::cout << "Iteration " << iter 
                  << " | Current Cost: " << calculatePenalizedCost(currentSol)
                  << " | Best Cost: " << calculatePenalizedCost(bestSol)
                  << " | Best Feasible Cost: " 
                  << (feasibleFound ? std::to_string(bestFeasibleSolution.rawObjectiveValue) : "N/A")
                  << std::endl;
        // Check Time Limit
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (timeLimit.has_value() && elapsed.count() > timeLimit.value()) {
            std::cout << "Time limit reached." << std::endl;
            break;
        }

        // 2. Find Best Neighbor [Section 4.2]
        TabuSolution nextSol = bestNeighbor(currentSol, iter);
        
        // 3. Update Current Solution
        currentSol = nextSol;
        evaluateSolution(currentSol);

        // Update Global Best (Minimize f(s))
        if (calculatePenalizedCost(currentSol) < calculatePenalizedCost(bestSol)) {
            bestSol = currentSol;
        }

        // Update Feasible Best
        if (currentSol.isFeasible()) {
            if (!feasibleFound || currentSol.rawObjectiveValue < bestFeasibleSolution.rawObjectiveValue) {
                bestFeasibleSolution = currentSol;
                feasibleFound = true;
                std::cout << "New Best Feasible Cost: " << bestFeasibleSolution.rawObjectiveValue 
                          << " at iter " << iter << std::endl;
            }
        }

        // 4. Update Penalty Parameters [Section 4.5]
        /*
        if (currentSol.totalLoadV == 0) alpha /= (1 + delta); else alpha *= (1 + delta);
        if (currentSol.totalDurationV == 0) beta /= (1 + delta); else beta *= (1 + delta);
        if (currentSol.totalTWV == 0) gamma /= (1 + delta); else gamma *= (1 + delta);
        if (currentSol.totalRideV == 0) tau /= (1 + delta); else tau *= (1 + delta);
        */

        alpha = std::clamp(alpha, MIN_PENALTY, MAX_PENALTY);
        beta  = std::clamp(beta,  MIN_PENALTY, MAX_PENALTY);
        gamma = std::clamp(gamma, MIN_PENALTY, MAX_PENALTY);
        tau   = std::clamp(tau,   MIN_PENALTY, MAX_PENALTY);

        iter++;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = end - start;
    finalSolveTime = totalElapsed.count();
}

// ---------------------------------------------------------
// Initial Solution Generation [Section 4.4]
// ---------------------------------------------------------
TabuSolution DARPMDTabuSolver::generateInitialSolution() {
    TabuSolution sol;
    sol.routes.resize(data.K.size());

    // Initialize empty routes with Start/End depots
    for (size_t k_idx = 0; k_idx < data.K.size(); ++k_idx) {
        int k = data.K[k_idx];
        sol.routes[k_idx].vehicleId = k;
        sol.routes[k_idx].sequence.push_back(data.StartNode.at(k));
        sol.routes[k_idx].sequence.push_back(data.EndNode.at(k));
    }

    // Randomly assign requests
    std::vector<int> requests = data.P;
    std::shuffle(requests.begin(), requests.end(), rng);

    for (int p : requests) {
        int d = getDeliveryNode(p);
        
        // Pick random vehicle
        std::uniform_int_distribution<> dist(0, data.K.size() - 1);
        int vIdx = dist(rng);

        // Insert at the end (before the final depot)
        auto& seq = sol.routes[vIdx].sequence;
        // Insert Pickup before EndNode
        seq.insert(seq.end() - 1, p);
        // Insert Delivery before EndNode (after Pickup)
        seq.insert(seq.end() - 1, d);
        
        // Track frequency
        freqMatrix[p][data.K[vIdx]]++;
    }

    return sol;
}

// ---------------------------------------------------------
// Neighborhood Search [Section 4.2]
// ---------------------------------------------------------
TabuSolution DARPMDTabuSolver::bestNeighbor(const TabuSolution& currentSol, int iter) {
    TabuSolution bestNeighborSol = currentSol;
    double bestNeighborCost = std::numeric_limits<double>::max();
    bool moveFound = false;

    // Try moving every request 'req' from its current route to every other route 'k_dest'
    for (int req : data.P) {
        // Find current vehicle of request
        int currentVIdx = -1;
        int pickupPos = -1;
        int deliveryPos = -1;
        int deliveryNode = getDeliveryNode(req);

        for (size_t k = 0; k < currentSol.routes.size(); ++k) {
            auto itP = std::find(currentSol.routes[k].sequence.begin(), currentSol.routes[k].sequence.end(), req);
            if (itP != currentSol.routes[k].sequence.end()) {
                currentVIdx = k;
                pickupPos = std::distance(currentSol.routes[k].sequence.begin(), itP);
                
                auto itD = std::find(currentSol.routes[k].sequence.begin(), currentSol.routes[k].sequence.end(), deliveryNode);
                deliveryPos = std::distance(currentSol.routes[k].sequence.begin(), itD);
                break;
            }
        }

        if (currentVIdx == -1) continue; // Should not happen

        // Try moving to every other vehicle
        for (size_t destVIdx = 0; destVIdx < data.K.size(); ++destVIdx) {
            if (currentVIdx == (int)destVIdx) continue;

            int vehicleId = data.K[destVIdx];

            // Tabu Check [Section 4.2]
            // If we move req to vehicleId, is it tabu?
            // "When request i is removed from route k, reinsertion in k is forbidden"
            // The paper says: attribute (i, k) is tabu. 
            // In this implementation context: Moving i TO destVIdx.
            bool isTabu = (tabuList[req][vehicleId] > iter);

            // Create candidate solution (copy)
            // Ideally, optimize this to not do full deep copy for every check
            TabuSolution candidate = currentSol;
            
            // 1. Remove from current route
            auto& srcSeq = candidate.routes[currentVIdx].sequence;
            // Remove Delivery first (higher index)
            srcSeq.erase(srcSeq.begin() + deliveryPos);
            srcSeq.erase(srcSeq.begin() + pickupPos);

            // 2. Insert into dest route
            auto& destSeq = candidate.routes[destVIdx].sequence;
            
            // Simple insertion strategy: Try all valid pair positions O(N^2)
            // The paper suggests determining best position for critical vertex first.
            // For robustness here, we iterate all valid positions i < j.
            
            double localBestCost = std::numeric_limits<double>::max();
            TabuRoute bestRouteConfig = candidate.routes[destVIdx];
            bool configFound = false;

            // Start from 1 (after depot) to size-1 (before depot)
            // Current size is N (including start/end). New size will be N+2.
            // Insertion logic:
            // Route: S ... E. Size = M.
            // Valid insert indices for P: 1 to M-1.
            // Valid insert indices for D: p_idx + 1 to M.
            
            int M = destSeq.size(); 
            for (int i = 1; i < M; ++i) {
                for (int j = i + 1; j <= M; ++j) {
                    TabuRoute tempRoute = candidate.routes[destVIdx];
                    
                    // Insert Pickup at i
                    tempRoute.sequence.insert(tempRoute.sequence.begin() + i, req);
                    // Insert Delivery at j (index shifts by 1 because of pickup insertion)
                    tempRoute.sequence.insert(tempRoute.sequence.begin() + j, deliveryNode);
                    
                    evaluateRoute(tempRoute);
                    
                    // Calculate cost impact just for this route change
                    // Cost = c(s) + weights * violations
                    double rtCost = tempRoute.routeCost + 
                                    alpha * tempRoute.loadViolation + 
                                    beta * tempRoute.durationViolation + 
                                    gamma * tempRoute.timeWindowViolation + 
                                    tau * tempRoute.rideTimeViolation;
                                    
                    if (rtCost < localBestCost) {
                        localBestCost = rtCost;
                        bestRouteConfig = tempRoute;
                        configFound = true;
                    }
                }
            }
            
            if (configFound) {
                candidate.routes[destVIdx] = bestRouteConfig;
                // Evaluate the removed route as well to update its stats
                evaluateRoute(candidate.routes[currentVIdx]);
                
                // Recalculate global metrics
                evaluateSolution(candidate);
                
                double penalizedCost = calculatePenalizedCost(candidate);
                double diversification = calculateDiversificationPenalty(candidate, req, vehicleId);
                double totalObj = penalizedCost + diversification;

                // Aspiration Criterion [Section 4.2]
                bool aspiration = (penalizedCost < calculatePenalizedCost(bestFeasibleSolution));
                
                if (!isTabu || aspiration) {
                    if (totalObj < bestNeighborCost) {
                        bestNeighborCost = totalObj;
                        bestNeighborSol = candidate;
                        moveFound = true;
                        
                        // Set Tabu for the NEXT iteration:
                        // "When request i is removed from route k, reinsertion in k is forbidden"
                        // Here we removed from currentVIdx (ID: data.K[currentVIdx])
                        // So (req, data.K[currentVIdx]) becomes Tabu.
                        // NOTE: In the loop, we select the BEST. We apply tabu update outside loop.
                    }
                }
            }
        }
    }
    
    // Apply Tabu Logic for the chosen move
    if (moveFound) {
        // Identify the move performed
        // We need to know which request moved from where to where.
        // Doing a diff or storing the move details is better.
        // For simplicity, we re-scan to find the move details (inefficient but clear).
        
        for (int req : data.P) {
            int oldV = -1, newV = -1;
            // Find in current
            for(size_t k=0; k<currentSol.routes.size(); ++k) 
                for(int n : currentSol.routes[k].sequence) if(n==req) oldV = data.K[k];
            
            // Find in new
            for(size_t k=0; k<bestNeighborSol.routes.size(); ++k) 
                for(int n : bestNeighborSol.routes[k].sequence) if(n==req) newV = data.K[k];
                
            if (oldV != -1 && newV != -1 && oldV != newV) {
                // Request 'req' moved from oldV to newV.
                // Ban re-inserting 'req' back into 'oldV'.
                tabuList[req][oldV] = iter + tabuTenure;
                
                // Update frequency
                freqMatrix[req][newV]++;
                break;
            }
        }
    }
    
    return bestNeighborSol;
}

// ---------------------------------------------------------
// Objective Functions
// ---------------------------------------------------------
double DARPMDTabuSolver::calculatePenalizedCost(const TabuSolution& sol) {
    return sol.rawObjectiveValue + 
           alpha * sol.totalLoadV + 
           beta * sol.totalDurationV + 
           gamma * sol.totalTWV + 
           tau * sol.totalRideV;
}

double DARPMDTabuSolver::calculateDiversificationPenalty(const TabuSolution& sol, int reqId, int vehicleId) {
    // p(s) = lambda * c(s) * sqrt(nm) * rho_ik [Section 4.3]
    // assuming standard lambda = 0.015
    double lambda = 0.015;
    double nm = std::sqrt(data.P.size() * data.K.size());
    int rho = freqMatrix[reqId][vehicleId];
    return lambda * sol.rawObjectiveValue * nm * rho;
}

// ---------------------------------------------------------
// Route Evaluation - The 8-Step Procedure [Section 4.7]
// ---------------------------------------------------------
void DARPMDTabuSolver::evaluateSolution(TabuSolution& sol) {
    sol.rawObjectiveValue = 0;
    sol.totalLoadV = 0;
    sol.totalDurationV = 0;
    sol.totalTWV = 0;
    sol.totalRideV = 0;

    for (auto& route : sol.routes) {
        evaluateRoute(route);
        sol.rawObjectiveValue += route.routeCost;
        sol.totalLoadV += route.loadViolation;
        sol.totalDurationV += route.durationViolation;
        sol.totalTWV += route.timeWindowViolation;
        sol.totalRideV += route.rideTimeViolation;
    }
}

void DARPMDTabuSolver::evaluateRoute(TabuRoute& route) {
    // Reset metrics
    route.routeCost = 0;
    route.loadViolation = 0;
    route.durationViolation = 0;
    route.timeWindowViolation = 0;
    route.rideTimeViolation = 0;
    
    if (route.sequence.empty()) return;

    int veh = route.vehicleId;
    double T_max = data.max_route_time.at(veh);
    double Q_max = data.capacity.at(veh);
    double L_max = data.max_ride_time;

    // --- 1. Basic Routing Cost & Load Check ---
    double currentLoad = 0;
    for (size_t i = 0; i < route.sequence.size() - 1; ++i) {
        int u = route.sequence[i];
        int v = route.sequence[i+1];
        route.routeCost += data.getCost(u, v, veh);
        
        // Load at node u (before leaving)
        if (i > 0) { // Don't count load at start depot
             // Check capacity logic: 
             // StartNode -> load=0. 
             // Visit P -> load++. Visit D -> load--.
             // In provided ILP: q=demand.
             double demand = data.getDemand(u);
             currentLoad += demand;
             if (currentLoad > Q_max) route.loadViolation += (currentLoad - Q_max);
             if (currentLoad < 0) route.loadViolation += std::abs(currentLoad); // Should be 0
        }
    }

    // --- The 8-Step Forward Time Slack Algorithm ---
    
    // Variables for the route nodes
    int n = route.sequence.size();
    std::vector<double> A(n), W(n), B(n), D(n);
    // Maps for easy lookup later
    std::map<int, int> nodeToIndex;
    for(int i=0; i<n; ++i) nodeToIndex[route.sequence[i]] = i;

    auto calcTimes = [&](double start_time) {
        D[0] = start_time;
        for (int i = 0; i < n - 1; ++i) {
            int u = route.sequence[i];
            int v = route.sequence[i+1];
            double t_uv = data.getTravelTime(u, v);
            double serv_u = (i == 0) ? 0 : data.getServiceTime(u);
            
            A[i+1] = D[i] + t_uv; // Arrival at next
            
            double e_v = data.getTimeWindowStart(v);
            double l_v = data.getTimeWindowEnd(v);
            
            // Wait if early
            W[i+1] = std::max(0.0, e_v - A[i+1]);
            B[i+1] = A[i+1] + W[i+1]; // Begin Service
            
            // Departure from v
            double serv_v = (i+1 == n-1) ? 0 : data.getServiceTime(v);
            D[i+1] = B[i+1] + serv_v;
        }
    };

    // Step 1: Set D_0 = e_0
    double e_depot = data.getTimeWindowStart(route.sequence[0]);
    calcTimes(e_depot);

    // Step 2 (Implicit in calcTimes): A, W, B, D computed.

    // Step 3: Compute F_0 (Forward Slack at Depot)
    // F_i = min_{i <= j <= q} { sum_{i < p <= j} W_p + (l_j - B_j)^+ }
    
    auto computeForwardSlack = [&](int i) -> double {
        double min_val = std::numeric_limits<double>::max();
        double sum_W = 0;
        for (int j = i; j < n; ++j) {
            int node_j = route.sequence[j];
            double l_j = data.getTimeWindowEnd(node_j);
            
            if (j > i) sum_W += W[j];
            
            // Ride time constraint impact on slack (Eq 5 in paper)
            double ride_constraint_term = std::numeric_limits<double>::max();
            
            // If j is a destination for a request picked up AFTER i
            if (isDelivery(node_j)) {
                // Find pickup
                // Note: simplistic lookup here.
                // P_j is ride time of user whose dest is v_j.
                // Paper: min{ l_j - B_j, L - P_j }
                // Implementation complex without direct P_j link, skipping precise ride-time slack integration 
                // for this simplified implementation, sticking to TW slack:
            }

            double term = sum_W + std::max(0.0, l_j - B[j]);
            if (term < min_val) min_val = term;
        }
        return min_val;
    };

    double F_0 = computeForwardSlack(0);

    // Step 4: Delay Depot Departure
    // D_0 := e_0 + min(F_0, sum W_p)
    double total_waiting = 0;
    for(double w : W) total_waiting += w;
    
    double new_start = e_depot + std::min(F_0, total_waiting);
    
    // Step 5: Update times
    calcTimes(new_start);

    // Step 6: Compute Ride Times L_i initially
    // Calculated below in violation check

    // Step 7: Minimize Ride Times by delaying service at origins
    // For every vertex v_j that is an Origin:
    for (int j = 1; j < n - 1; ++j) {
        int node = route.sequence[j];
        if (isPickup(node)) {
            // Compute F_j (simplified to TW slack for speed)
            double F_j = computeForwardSlack(j);
            
            // Shift B_j
            double sum_W_after = 0;
            for(int k=j+1; k<n; ++k) sum_W_after += W[k];
            
            double shift = std::min(F_j, sum_W_after);
            if (shift > 0) {
                B[j] += shift;
                D[j] = B[j] + data.getServiceTime(node);
                
                // Propagate changes forward
                // This is O(N^2) total inside route eval.
                for (int k = j; k < n - 1; ++k) {
                   int u = route.sequence[k];
                   int v = route.sequence[k+1];
                   double t_uv = data.getTravelTime(u, v);
                   A[k+1] = D[k] + t_uv;
                   double e_v = data.getTimeWindowStart(v);
                   W[k+1] = std::max(0.0, e_v - A[k+1]);
                   B[k+1] = A[k+1] + W[k+1];
                   double serv_v = (k+1 == n-1) ? 0 : data.getServiceTime(v);
                   D[k+1] = B[k+1] + serv_v;
                }
            }
        }
    }

    // Step 8: Compute Violations
    
    // Duration
    double duration = A[n-1] - D[0]; // Arrival at end depot - Departure from start
    if (duration > T_max) route.durationViolation = duration - T_max;

    // Time Windows
    for (int i = 0; i < n; ++i) {
        int node = route.sequence[i];
        double l_i = data.getTimeWindowEnd(node);
        if (B[i] > l_i) route.timeWindowViolation += (B[i] - l_i);
        
        // Save to struct for results
        route.arrivalTimes[node] = A[i];
        route.beginServiceTimes[node] = B[i];
        route.waitTimes[node] = W[i];
    }

    // Ride Times
    for (int i = 1; i < n - 1; ++i) {
        int u = route.sequence[i];
        if (isPickup(u)) {
            int d = getDeliveryNode(u);
            if (nodeToIndex.find(d) != nodeToIndex.end()) {
                int d_idx = nodeToIndex[d];
                // Ride time = B_dest - D_origin [Source 119]
                double rideTime = B[d_idx] - D[i];
                route.rideTimes[u] = rideTime;
                
                if (rideTime > L_max) route.rideTimeViolation += (rideTime - L_max);
            }
        }
    }
}

// ---------------------------------------------------------
// Helpers
// ---------------------------------------------------------
int DARPMDTabuSolver::getDeliveryNode(int pickupNode) const {
    // Assuming structure: requests are 1..N or 0..N-1.
    // The ILP code used: i + data.N_requests.
    // We strictly assume this relationship holds for the dataset.
    return pickupNode + data.N_requests;
}

bool DARPMDTabuSolver::isPickup(int node) const {
    return std::find(data.P.begin(), data.P.end(), node) != data.P.end();
}

bool DARPMDTabuSolver::isDelivery(int node) const {
    return std::find(data.D.begin(), data.D.end(), node) != data.D.end();
}

DARPMD_ResultInstance DARPMDTabuSolver::getResult() const {
    DARPMD_ResultInstance result(data);
    result.solveTime = finalSolveTime;
    
    // Determine which solution to report
    const TabuSolution* solToReport = nullptr;
    if (feasibleFound) {
        result.solverStatus = "Feasible";
        solToReport = &bestFeasibleSolution;
        std::cout << "\n=== FACTIBLE SOLUTION FOUND ===" << std::endl;
    } else {
        result.solverStatus = "Infeasible";
        solToReport = &bestSol; // Report best found even if infeasible
        std::cout << "\n=== NO FACTIBLE SOLUTION ENCONTRADA ===" << std::endl;
    }

    auto calculatePenalizedCost = [&](const TabuSolution& sol) {
        return sol.rawObjectiveValue + 
               alpha * sol.totalLoadV + 
               beta * sol.totalDurationV + 
               gamma * sol.totalTWV + 
               tau * sol.totalRideV;
    };

    std::cout << "Costo de Rutas (Distancia/Tiempo): " << solToReport->rawObjectiveValue << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Violación Carga (Load):       " << solToReport->totalLoadV << std::endl;
    std::cout << "Violación Duración Ruta:      " << solToReport->totalDurationV << std::endl;
    std::cout << "Violación Ventanas de Tiempo: " << solToReport->totalTWV << std::endl;
    std::cout << "Violación Tiempo de Viaje:    " << solToReport->totalRideV << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Costo Penalizado Total:       " << calculatePenalizedCost(*solToReport) << std::endl;
    
    result.objectiveValue = solToReport->rawObjectiveValue;

    for (const auto& r : solToReport->routes) {
        VehicleRoute vr;
        vr.vehicleId = r.vehicleId;
        
        for (int node : r.sequence) {
            RouteStep step;
            step.nodeId = node;
            
            if (node == data.StartNode.at(r.vehicleId)) step.type = "DepotStart";
            else if (node == data.EndNode.at(r.vehicleId)) step.type = "DepotEnd";
            else if (isPickup(node)) step.type = "Pickup";
            else step.type = "Delivery";
            
            if (r.arrivalTimes.count(node)) 
                step.arrivalTime = r.beginServiceTimes.at(node); // Using B_i as service time
            else step.arrivalTime = 0.0;
            
            // Load logic could be re-simulated here if needed for the ResultInstance
            step.loadAfter = 0.0; // Placeholder
            
            vr.steps.push_back(step);
        }
        result.addRoute(r.vehicleId, vr);
    }

    return result;
}