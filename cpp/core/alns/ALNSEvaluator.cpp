#include "ALNSEvaluator.h"

#include "../ALNSSolver.h"

ALNSEvaluator::ALNSEvaluator(const DARPMD_ProblemInstance& data, 
                             const ALNSParams& params)
    : data(data), params(params) { };

void ALNSEvaluator::evaluateRoute(ALNSRoute& route) {
    // Reset metrics
    route.distanceCost = 0.0;
    route.timeWindowViolation = 0.0;
    route.vehicleMaxRouteTimeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;

    if (route.sequence.empty()) return;

    int q = route.sequence.size() - 1; // Last index (end depot)
    if ((int)route.arrivalTimes.size() < data.max_node_id + 1) {
        route.resize(data.max_node_id + 1);
    }

    // Temporary arrays for the evaluation procedure
    std::vector<double>& A = route.A;
    std::vector<double>& W = route.W;
    std::vector<double>& B = route.B;
    std::vector<double>& D = route.D;
    std::vector<double>& Fi = route.Fi;
    std::vector<int>& id2pos = route.id2pos;
    if (route.sequence.size() != A.size()) {
        A.resize(route.sequence.size());
        W.resize(route.sequence.size());
        B.resize(route.sequence.size());
        D.resize(route.sequence.size());
        Fi.resize(route.sequence.size());
    }
    if (route.id2pos.size() != data.max_node_id + 1) {
        route.id2pos.resize(data.max_node_id + 1, -1);
    }

    std::vector<double> pickup_D(data.N_requests + 1, -1.0);

    for (int i = 0; i <= q; ++i) {
        int v = route.sequence[i];
        id2pos[v] = i;
    }

    // Helper Lambda: Propagate times forward from a given index
    auto propagateForward = [&](int startIndex) {
        for (int i = startIndex; i <= q; ++i) {
            int u = route.sequence[i - 1];
            int v = route.sequence[i];
            
            A[i] = D[i - 1] + data.getTravelTime(u, v);
            double earlyTW = data.getTimeWindowStart(v);
            
            W[i] = std::max(0.0, earlyTW - A[i]);
            B[i] = A[i] + W[i];
            D[i] = B[i] + data.getServiceTime(v);

            if (data.isPickup(v)) {
                pickup_D[v] = D[i];
            }
        }
    };

    // Helper Lambda: Calculate Forward Time Slack (F_i)
    auto calculateF = [&](int i) {
        double F = std::numeric_limits<double>::infinity();
        double sumW = 0.0;
        
        for (int j = i; j <= q; ++j) {
            if (j > i) {
                sumW += W[j];
            }
            int vj = route.sequence[j];
            double lj = data.getTimeWindowEnd(vj);
            double Pj = 0.0;
            
            // If vj is a delivery node, calculate current ride time (Pj)
            if (data.isDelivery(vj)) {
                int pickupId = vj - data.N_requests; // Assuming D_id = P_id + N_requests
                Pj = B[j] - pickup_D[pickupId]; // Ride time = Begin service at D - Departure at P
            }
            
            double timeWindowMargin = lj - B[j];
            double rideTimeMargin = data.getMaxRideTime() - Pj;
            
            double margin = std::max(0.0, std::min(timeWindowMargin, rideTimeMargin));
            F = std::min(F, sumW + margin);
        }
        return F;
    };

    // Helper: Calculate sum of waiting times between two indices exclusive of start, inclusive of end
    auto getSumW = [&](int start, int end) {
        double sum = 0.0;
        for (int p = start + 1; p < end; ++p) sum += W[p];
        return sum;
    };

    // PHASE 1: Time Window Minimization
    D[0] = std::max(0.0, data.getTimeWindowStart(route.sequence[0]));
    B[0] = D[0]; // Assuming depot service time is 0
    propagateForward(1);

    // PHASE 2: Route Duration Minimization
    double F0 = calculateF(0);
    double sumW_depot = getSumW(0, q);
    
    D[0] = data.getTimeWindowStart(route.sequence[0]) + std::min(F0, sumW_depot);
    propagateForward(1); // Re-calculate A, W, B, D

    // PHASE 3: Ride Time Minimization
    for (int j = 1; j < q; ++j) {
        int vj = route.sequence[j];
        if (data.isPickup(vj)) {
            double Fj = calculateF(j);
            double sumW_j = getSumW(j, q);
            
            double shift = std::min(Fj, sumW_j);
            if (shift > 0.0) {
                B[j] += shift;
                D[j] = B[j] + data.getServiceTime(vj);
                propagateForward(j + 1); // Propagate changes to the rest of the route
            }
        }
    }

    // PHASE 4: Final Evaluation & Constraint Checks
    double currentLoad = 0.0;
    double capacity = data.getVehicleCapacity(route.vehicleId);
    
    route.arrivalTimes[route.sequence[0]] = A[0];
    route.loads[route.sequence[0]] = 0.0;

    for (int i = 1; i <= q; ++i) {
        int u = route.sequence[i - 1];
        int v = route.sequence[i];

        // Assign definitive arrival times
        route.arrivalTimes[v] = A[i];

        // Distance Cost
        route.distanceCost += data.getCost(u, v, route.vehicleId);

        // Capacity Constraint
        currentLoad += data.getDemand(v);
        route.loads[v] = currentLoad;
        if (currentLoad > capacity) {
            route.loadViolation += (currentLoad - capacity);
        }

        // Time Window Constraint
        double lateTW = data.getTimeWindowEnd(v);
        if (B[i] > lateTW) {
            route.timeWindowViolation += (B[i] - lateTW);
        }

        // Ride Time Constraint
        if (data.isDelivery(v)) {
            int pickupId = v - data.N_requests;
            int pickupPos = id2pos[pickupId];
            double rideTime = B[i] - D[pickupPos];
            if (rideTime > data.getMaxRideTime()) {
                route.rideTimeViolation += (rideTime - data.getMaxRideTime());
            }
        }
    }

    // Max Route Duration Constraint
    double duration = A[q] - D[0]; // Arrival at end depot - Departure from start depot
    double maxRouteTime = data.getVehicleMaxRouteTime(route.vehicleId);
    if (duration > maxRouteTime) {
        route.vehicleMaxRouteTimeViolation += (duration - maxRouteTime);
    }

    // PHASE 5: calculate F_i for all nodes (for potential use in move evaluations)
    route.Fi.assign(q + 1, 0.0);
    for (int i = q; i >= 0; --i) {
        int v = route.sequence[i];
        
        double margin_TW = data.getTimeWindowEnd(v) - route.B[i];
        
        double margin_Ride = std::numeric_limits<double>::infinity();
        if (data.isDelivery(v)) {
            int pickupId = v - data.N_requests;
            int pickupPos = id2pos[pickupId];
            double rideTime = route.B[i] - route.D[pickupPos];
            margin_Ride = data.getMaxRideTime() - rideTime;
        }
        
        double local_margin = std::max(0.0, std::min(margin_TW, margin_Ride));
        
        if (i == q) {
            route.Fi[i] = local_margin;
        } else {
            route.Fi[i] = std::min(local_margin, route.W[i + 1] + route.Fi[i + 1]);
        }
    }


    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeWindowViolation
                    + params.vehicleMaxRouteTimePenalty * route.vehicleMaxRouteTimeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeWindowViolation == 0 && 
                        route.vehicleMaxRouteTimeViolation == 0 &&
                        route.loadViolation == 0 && 
                        route.rideTimeViolation == 0);
}

void ALNSEvaluator::evaluateRouteGreedy(ALNSRoute& route) {
    // Reset metrics
    route.distanceCost = 0.0;
    route.timeWindowViolation = 0.0;
    route.vehicleMaxRouteTimeViolation = 0.0;
    route.loadViolation = 0.0;
    route.rideTimeViolation = 0.0;

    if (route.sequence.empty()) return;

    if ((int)route.arrivalTimes.size() < data.max_node_id + 1) {
        route.resize(data.max_node_id + 1);
    }

    // Assumption: pickup ids 1..n
    static thread_local std::vector<double> pickupTimes;
    if ((int)pickupTimes.size() < data.N_requests + 1) pickupTimes.resize(data.N_requests + 1);
    std::fill(pickupTimes.begin(), pickupTimes.end(), -1.0);

    double currentTime = 0.0;
    double currentLoad = 0.0;
    
    // 1. Initialize at Start Depot
    int startNode = route.sequence.front();
    currentTime = std::max(0.0, data.getTimeWindowStart(startNode)); 
    
    route.arrivalTimes[startNode] = currentTime;
    route.loads[startNode] = 0.0;

    for (size_t i = 0; i < route.sequence.size() - 1; ++i) {
        int u = route.sequence[i];
        int v = route.sequence[i+1];

        // --- Distance ---
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
            route.timeWindowViolation += (arrivalAtV - lateTW);
            // In a soft TW context, we assume we arrive 'late' but continue
            // To prevent massive propagation, sometimes we clamp, but here we let it flow
        }

        currentTime = arrivalAtV;
        route.arrivalTimes[v] = currentTime;

        // --- Capacity ---
        double demand = data.getDemand(v);
        currentLoad += demand;
        route.loads[v] = currentLoad;

        double capacity = data.getVehicleCapacity(route.vehicleId);
        if (currentLoad > capacity) {
            route.loadViolation += (currentLoad - capacity);
        }

        // --- Ride Time Logic ---
        // If v is a pickup node (in P)
        if (data.isPickup(v)) {
            pickupTimes[v] = currentTime;
        }
        // If v is a delivery node (in D)
        else if (data.isDelivery(v)) {
            // Find corresponding pickup ID. Assuming D_id = P_id + N_requests
            int pickupId = v - data.N_requests;
            
            if (pickupTimes[pickupId] >= 0) {
                double rideTime = currentTime - (pickupTimes[pickupId] + data.getServiceTime(pickupId));
                if (rideTime > data.getMaxRideTime()) {
                    route.rideTimeViolation += (rideTime - data.getMaxRideTime());
                }
            }
        }
    }

    // Check Max Route Duration
    int endNode = route.sequence.back();
    double duration = route.arrivalTimes[endNode] - route.arrivalTimes[startNode];
    if (duration > data.getVehicleMaxRouteTime(route.vehicleId)) {
        route.vehicleMaxRouteTimeViolation += (duration - data.getVehicleMaxRouteTime(route.vehicleId));
    }

    // Final Cost Aggregation
    route.totalCost = route.distanceCost 
                    + params.timeWindowPenalty * route.timeWindowViolation
                    + params.vehicleMaxRouteTimePenalty * route.vehicleMaxRouteTimeViolation
                    + params.capacityPenalty * route.loadViolation
                    + params.rideTimePenalty * route.rideTimeViolation;

    route.isFeasible = (route.timeWindowViolation == 0 && 
                        route.vehicleMaxRouteTimeViolation == 0 &&
                        route.loadViolation == 0 && 
                        route.rideTimeViolation == 0);
}

void ALNSEvaluator::evaluateSolutionGreedy(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRouteGreedy(r);
        sol.objectiveValue += r.totalCost;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

void ALNSEvaluator::evaluateSolution(ALNSSolution& sol) {
    sol.objectiveValue = 0.0;
    for (auto& r : sol.routes) {
        evaluateRoute(r);
        sol.objectiveValue += r.totalCost;
    }
    // Penalize unassigned (the unassigned request set must be handled by the operators)
    sol.objectiveValue += sol.unassignedRequests.size() * params.unassignedPenalty;
}

bool ALNSEvaluator::solutionHasViolations(const ALNSSolution& sol) const {
    for (const auto& r : sol.routes) {
        if (!r.isFeasible) return true;
    }
    return false;
}

ALNSEvaluator::InsertionMove ALNSEvaluator::findBestInsertion(ALNSRoute& route, int requestId) {
    InsertionMove bestMove = {-1, -1, std::numeric_limits<double>::infinity()};
    
    int q = route.sequence.size() - 1;
    double demand = data.getDemand(requestId); // Pickup demand
    double capacity = data.getVehicleCapacity(route.vehicleId);

    const std::vector<double>& A = route.A;
    const std::vector<double>& W = route.W;
    const std::vector<double>& B = route.B;
    const std::vector<double>& D = route.D;

    // Iterar todas las posiciones posibles para el Pickup
    for (int i = 1; i <= q; ++i) {
        
        // 1. Calcular Delta de Distancia local para el Pickup
        int prev_i = route.sequence[i - 1];
        int next_i = route.sequence[i];
        double distShiftP = data.getCost(prev_i, requestId, route.vehicleId) + 
                            data.getCost(requestId, next_i, route.vehicleId) - 
                            data.getCost(prev_i, next_i, route.vehicleId);

        // 2. Calcular Retraso (Shift) generado en el nodo i
        double A_P = route.D[i - 1] + data.getTravelTime(prev_i, requestId);
        double B_P = std::max(A_P, data.getTimeWindowStart(requestId));
        double D_P = B_P + data.getServiceTime(requestId);
        
        double A_next_i_new = D_P + data.getTravelTime(requestId, next_i);
        double shift_i = std::max(0.0, A_next_i_new - route.B[i]);

        // Heurística rápida: Si el shift rompe masivamente el F_i, sabemos que habrá mucha penalización
        double timePenaltyEstimateP = std::max(0.0, shift_i - route.Fi[i]) * params.timeWindowPenalty;

        // Iterar todas las posiciones posibles para el Delivery (j >= i)
        for (int j = i; j <= q; ++j) {
            
            // Check rápido de capacidad: Si en algún punto entre i y j la carga original + nueva demanda > capacidad
            // (Idealmente esto se optimiza para no hacer un bucle interno)
            bool loadViolated = false;
            double extraLoadPenalty = 0.0;
            for(int k = i; k < j; ++k) {
                if (route.loads[route.sequence[k]] + demand > capacity) {
                    extraLoadPenalty += (route.loads[route.sequence[k]] + demand - capacity) * params.capacityPenalty;
                }
            }

            double distShiftD = 0.0;
            if (i == j) {
                // Caso especial: Pickup y Delivery adyacentes
                distShiftD = data.getCost(prev_i, requestId, route.vehicleId) + 
                             data.getCost(requestId, requestId + data.N_requests, route.vehicleId) + 
                             data.getCost(requestId + data.N_requests, next_i, route.vehicleId) - 
                             data.getCost(prev_i, next_i, route.vehicleId);
            } else {
                int prev_j = route.sequence[j - 1];
                int next_j = route.sequence[j];
                distShiftD = distShiftP + 
                             data.getCost(prev_j, requestId + data.N_requests, route.vehicleId) + 
                             data.getCost(requestId + data.N_requests, next_j, route.vehicleId) - 
                             data.getCost(prev_j, next_j, route.vehicleId);
            }

            // Estimación de coste total (Distancia + Estimación de Tiempo + Estimación Carga)
            double estimatedDeltaCost = distShiftD + timePenaltyEstimateP + extraLoadPenalty;

            // Si la estimación ya es peor que nuestro bestMove, lo descartamos inmediatamente O(1)
            if (estimatedDeltaCost < bestMove.deltaCost) {
                // AQUÍ: Como es prometedor, podemos hacer una propagación parcial exacta 
                // para calcular las penalizaciones reales de RideTime y TimeWindows.
                double exactDeltaCost = calculateExactInsertionDelta(route, requestId, i, j);
                //double exactDeltaCost = estimatedDeltaCost; 
                
                if (exactDeltaCost < bestMove.deltaCost) {
                    bestMove.deltaCost = exactDeltaCost;
                    bestMove.pickupPos = i;
                    bestMove.deliveryPos = j;
                }
            }
        }
    }
    return bestMove;
}

double ALNSEvaluator::calculateExactInsertionDelta(const ALNSRoute& route, int requestId, int i, int j) {
    int pickupNode = requestId;
    int deliveryNode = requestId + data.N_requests;
    double capacity = data.getVehicleCapacity(route.vehicleId);
    
    int orig_q = route.sequence.size() - 1;
    int new_q = orig_q + 2; // La ruta virtual tendrá 2 nodos más

    // --- 1. AHORRO / INCREMENTO DE DISTANCIA ---
    double distDelta = 0.0;
    int prev_i = route.sequence[i - 1];
    int next_i = route.sequence[i];

    if (i == j) {
        // Inserción adyacente: prev_i -> P -> D -> next_i
        distDelta = data.getCost(prev_i, pickupNode, route.vehicleId) + 
                    data.getCost(pickupNode, deliveryNode, route.vehicleId) + 
                    data.getCost(deliveryNode, next_i, route.vehicleId) - 
                    data.getCost(prev_i, next_i, route.vehicleId);
    } else {
        // Inserción separada
        int prev_j = route.sequence[j - 1];
        int next_j = route.sequence[j];
        
        distDelta = (data.getCost(prev_i, pickupNode, route.vehicleId) + 
                     data.getCost(pickupNode, next_i, route.vehicleId) - 
                     data.getCost(prev_i, next_i, route.vehicleId)) 
                  + 
                    (data.getCost(prev_j, deliveryNode, route.vehicleId) + 
                     data.getCost(deliveryNode, next_j, route.vehicleId) - 
                     data.getCost(prev_j, next_j, route.vehicleId));
    }

    // --- 2. RESTAR PENALIZACIONES ANTIGUAS (desde el nodo i) ---
    // Calculamos qué violaciones ya existían en la ruta original desde el punto de corte.
    double old_TW_viol = 0.0, old_Load_viol = 0.0, old_RT_viol = 0.0;
    
    for (int k = i; k <= orig_q; ++k) {
        int v = route.sequence[k];
        
        // Carga antigua
        if (route.loads[v] > capacity) old_Load_viol += (route.loads[v] - capacity);
        
        // TW antigua
        double lateTW = data.getTimeWindowEnd(v);
        if (route.B[k] > lateTW) old_TW_viol += (route.B[k] - lateTW);
        
        // Ride Time antiguo
        if (data.isDelivery(v)) {
            int p_id = v - data.N_requests;
            // Se asume que tienes Id2Pos_Map disponible en la clase o lo calculas
            int p_pos = route.id2pos[p_id]; 
            double rt = route.B[k] - route.D[p_pos];
            if (rt > data.getMaxRideTime()) old_RT_viol += (rt - data.getMaxRideTime());
        }
    }
    double old_Duration = route.A[orig_q] - route.D[0];
    double max_RouteTime = data.getVehicleMaxRouteTime(route.vehicleId);
    double old_Dur_viol = std::max(0.0, old_Duration - max_RouteTime);

    // --- 3. SIMULAR NUEVA RUTA Y SUMAR PENALIZACIONES NUEVAS ---
    // Mapeador mágico: Convierte un índice virtual (de la nueva ruta) en el ID del nodo
    auto getVirtualNode = [&](int idx) -> int {
        if (idx < i) return route.sequence[idx];
        if (idx == i) return pickupNode;
        if (idx > i && idx <= j) return route.sequence[idx - 1];
        if (idx == j + 1) return deliveryNode;
        return route.sequence[idx - 2];
    };

    double new_TW_viol = 0.0, new_Load_viol = 0.0, new_RT_viol = 0.0;
    
    // Estado inicial en el nodo previo a la inserción (i-1)
    double current_D = route.D[i - 1];
    double current_Load = route.loads[route.sequence[i - 1]];
    
    // Vector ligero para guardar los Departure Times nuevos y usarlos en el Ride Time
    std::vector<double> virtual_D(new_q + 1, 0.0);
    
    // Bucle de propagación O(N) desde la inserción hasta el final
    for (int k = i; k <= new_q; ++k) {
        int u = getVirtualNode(k - 1);
        int v = getVirtualNode(k);

        // -- Simular Carga --
        current_Load += data.getDemand(v);
        if (current_Load > capacity) new_Load_viol += (current_Load - capacity);

        // -- Simular Tiempos --
        double A = current_D + data.getTravelTime(u, v);
        double W = std::max(0.0, data.getTimeWindowStart(v) - A);
        double B = A + W;
        current_D = B + data.getServiceTime(v);
        virtual_D[k] = current_D; // Lo guardamos para los deliveries futuros

        double lateTW = data.getTimeWindowEnd(v);
        if (B > lateTW) new_TW_viol += (B - lateTW);

        // -- Simular Ride Time --
        if (data.isDelivery(v)) {
            int p_id = v - data.N_requests;
            double dep_P = 0.0;
            
            if (p_id == pickupNode) {
                dep_P = virtual_D[i]; // El P insertado está siempre en la pos virtual i
            } else {
                int orig_p_pos = route.id2pos[p_id];
                if (orig_p_pos < i) {
                    dep_P = route.D[orig_p_pos]; // Su pickup ocurrió antes del corte, tiempo intacto
                } else {
                    // Su pickup está en la zona modificada, buscamos su índice virtual
                    int shift = (orig_p_pos >= j) ? 2 : 1;
                    dep_P = virtual_D[orig_p_pos + shift];
                }
            }
            
            double rt = B - dep_P;
            if (rt > data.getMaxRideTime()) new_RT_viol += (rt - data.getMaxRideTime());
        }
    }
    
    double new_Duration = virtual_D[new_q] - route.D[0];
    double new_Dur_viol = std::max(0.0, new_Duration - max_RouteTime);

    // --- 4. CALCULAR COSTE FINAL ---
    double penaltyDelta = params.timeWindowPenalty * (new_TW_viol - old_TW_viol)
                        + params.capacityPenalty * (new_Load_viol - old_Load_viol)
                        + params.rideTimePenalty * (new_RT_viol - old_RT_viol)
                        + params.vehicleMaxRouteTimePenalty * (new_Dur_viol - old_Dur_viol);

    return distDelta + penaltyDelta;
}