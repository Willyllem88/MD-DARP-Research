double ALNSOperators::calculateInsertionCost(const ALNSRoute& route, size_t reqId, size_t pIdx, size_t dIdx) {
    if (route.sequence.empty()) return std::numeric_limits<double>::infinity();

    size_t deliveryId = reqId + data.N_requests;
    size_t newSize = route.sequence.size() + 2;

    // --- MAGIA DE RENDIMIENTO: Lambda para acceder a la ruta simulada en O(1) sin copiar memoria ---
    auto getVirtualNode = [&](size_t k) -> int {
        if (k == pIdx) return reqId;
        if (k == dIdx) return deliveryId;
        
        int insertedNodes = 0;
        if (k > pIdx) insertedNodes++;
        if (k > dIdx) insertedNodes++;
        
        return route.sequence[k - insertedNodes];
    };

    // Buffer hiper-rápido y persistente para los Ride Times.
    // Solo se inicializa una vez en toda la ejecución del programa por cada hilo.
    static thread_local std::vector<double> simPickupTimes;
    if ((int)simPickupTimes.size() < data.N_requests + 1) {
        simPickupTimes.assign(data.N_requests + 1, -1.0);
    }

    // --- VARIABLES DE ESTADO INICIAL ---
    int startNode = getVirtualNode(0); // Siempre es el Start Depot
    double currentTime = std::max(0.0, data.getTimeWindowStart(startNode));
    double startArrival = currentTime;
    double currentLoad = 0.0;
    double newDistanceCost = 0.0;

    int prevNode = startNode;

    // --- BUCLE DE SIMULACIÓN (Desde el nodo 1 hasta el End Depot) ---
    for (size_t k = 1; k < newSize; ++k) {
        int currNode = getVirtualNode(k);

        // 1. Coste y Distancia
        newDistanceCost += data.getCost(prevNode, currNode, route.vehicleId);

        // 2. Tiempo y Time Windows
        double travelTime = data.getTravelTime(prevNode, currNode);
        double serviceTime = data.getServiceTime(prevNode); 
        
        double arrival = currentTime + serviceTime + travelTime;
        double earlyTW = data.getTimeWindowStart(currNode);
        double lateTW = data.getTimeWindowEnd(currNode);

        // Si llegamos pronto, esperamos (Time Warp)
        if (arrival < earlyTW) {
            arrival = earlyTW;
        }

        // HARD CONSTRAINT: Si llegamos tarde, abortar inmediatamente
        if (arrival > lateTW) {
            goto INFEASIBLE; 
        }
        
        currentTime = arrival;

        // 3. Capacidad
        currentLoad += data.getDemand(currNode);
        if (currentLoad > data.getVehicleCapacity(route.vehicleId)) {
            goto INFEASIBLE;
        }

        // 4. Ride Time (Tiempos de viaje máximos para pasajeros)
        if (data.isPickup(currNode)) {
            simPickupTimes[currNode] = currentTime;
        } 
        else if (data.isDelivery(currNode)) {
            int pId = currNode - data.N_requests;
            
            // Verificamos si recogimos a este pasajero en la simulación
            if (simPickupTimes[pId] >= 0.0) {
                double rideTime = currentTime - (simPickupTimes[pId] + data.getServiceTime(pId));
                if (rideTime > data.getMaxRideTime()) {
                    goto INFEASIBLE;
                }
            }
        }

        prevNode = currNode;
    }

    // 5. Verificación Final: Max Route Duration
    if ((currentTime - startArrival) > data.getVehicleMaxRouteTime(route.vehicleId)) {
        goto INFEASIBLE;
    }

    // --- RUTA FACTIBLE: Limpiar entorno y retornar coste ---
    for (size_t k = 0; k < newSize; ++k) {
        int n = getVirtualNode(k);
        if (data.isPickup(n)) simPickupTimes[n] = -1.0;
    }
    
    return newDistanceCost - route.totalCost;


    // --- RUTA INFACTIBLE: Manejo centralizado (goto es estándar y eficiente aquí) ---
INFEASIBLE:
    // Limpiamos estrictamente solo los nodos que ensuciamos antes de romper
    for (size_t k = 0; k < newSize; ++k) {
        int n = getVirtualNode(k);
        if (data.isPickup(n)) simPickupTimes[n] = -1.0;
    }
    return std::numeric_limits<double>::infinity();
}


void ALNSOperators::repairRegret2(ALNSSolution& sol) {
    // Mientras queden peticiones sin asignar
    while (!sol.unassignedRequests.empty()) {
        
        int bestReqId = -1;
        double maxRegretValue = -1.0;
        
        // Estructura para almacenar el movimiento ganador de esta iteración
        struct WinningMove { 
            int vehicleIdx; 
            int pIdx; 
            int dIdx; 
            double costIncrease; 
        } bestMove = {-1, -1, -1, std::numeric_limits<double>::infinity()};

        // Copiamos el set a un vector para iterar
        std::vector<int> pending(sol.unassignedRequests.begin(), sol.unassignedRequests.end());

        // 1. EVALUAR CADA PETICIÓN PENDIENTE
        for (int reqId : pending) {
            
            // Estructura para guardar la mejor opción DE CADA VEHÍCULO
            struct VehicleOption { 
                int vIdx; 
                int pIdx; 
                int dIdx; 
                double cost; 
            };
            std::vector<VehicleOption> options;
            options.reserve(sol.routes.size());

            // 2. BUSCAR EL MEJOR HUECO EN CADA VEHÍCULO
            for (size_t v = 0; v < sol.routes.size(); ++v) {
                const auto& route = sol.routes[v];
                
                double bestCostForVehicle = std::numeric_limits<double>::infinity();
                int bestP = -1;
                int bestD = -1;

                // Optimización: Si el vehículo no puede ni con la demanda básica, saltar
                if (data.getDemand(reqId) > data.getVehicleCapacity(route.vehicleId)) continue;

                // Bucle de posiciones (Delta Evaluation)
                // i = posición de Pickup, j = posición de Delivery
                // Nota: i empieza en 1 (después del Start Depot)
                size_t currentSize = route.sequence.size();
                
                for (size_t i = 1; i <= currentSize; ++i) {
                    for (size_t j = i + 1; j <= currentSize; ++j) {
                        
                        // --- LA LLAMADA MÁGICA ---
                        // No crea objetos, no asigna memoria, retorna double o INF
                        double delta = calculateInsertionCost(route, reqId, i, j);

                        if (delta < bestCostForVehicle) {
                            bestCostForVehicle = delta;
                            bestP = (int)i;
                            bestD = (int)j;
                        }
                    }
                }

                // Si este vehículo encontró un hueco factible, lo guardamos como candidato
                if (bestCostForVehicle < std::numeric_limits<double>::infinity()) {
                    options.push_back({(int)v, bestP, bestD, bestCostForVehicle});
                }
            }

            // 3. CALCULAR EL REGRET PARA ESTA PETICIÓN
            if (options.empty()) continue; // Nadie puede llevarlo (infactible globalmente por ahora)

            // Ordenamos las opciones de menor a mayor coste
            std::sort(options.begin(), options.end(), [](const VehicleOption& a, const VehicleOption& b) {
                return a.cost < b.cost;
            });

            double regret = 0.0;
            if (options.size() == 1) {
                // Si solo cabe en uno, el arrepentimiento de no usarlo es "infinito"
                // Usamos un valor muy alto para priorizarlo sobre los demás
                regret = 1e9; 
            } else {
                // Regret-2 = (Coste de la 2ª mejor opción) - (Coste de la mejor opción)
                regret = options[1].cost - options[0].cost;
            }

            // 4. ¿ES ESTA LA PETICIÓN CON MAYOR ARREPENTIMIENTO?
            if (regret > maxRegretValue) {
                maxRegretValue = regret;
                bestReqId = reqId;
                
                // Guardamos los datos de la mejor opción (la posición 0 del vector ordenado)
                bestMove.vehicleIdx = options[0].vIdx;
                bestMove.pIdx = options[0].pIdx;
                bestMove.dIdx = options[0].dIdx;
                bestMove.costIncrease = options[0].cost;
            }
        }

        // 5. APLICAR EL MOVIMIENTO
        // Si no encontramos sitio para nadie, terminamos (no se puede reparar más)
        if (bestReqId == -1) break;

        // Modificamos FÍSICAMENTE la ruta (solo aquí, una vez por iteración del while)
        auto& route = sol.routes[bestMove.vehicleIdx];
        
        // Insertar Pickup
        route.sequence.insert(route.sequence.begin() + bestMove.pIdx, bestReqId);
        // Insertar Delivery
        int deliveryId = bestReqId + data.N_requests;
        route.sequence.insert(route.sequence.begin() + bestMove.dIdx, deliveryId);

        // Actualizamos los costes y tiempos reales de la ruta modificada
        // Esto es necesario para que la siguiente iteración tenga datos base correctos
        evaluator.evaluateRoute(route); 
        // Nota: evaluateRoute recalculará el totalCost, que debería coincidir 
        // con (coste_anterior + bestMove.costIncrease), pero es más seguro recalcular.

        // Marcar como asignado
        sol.unassignedRequests.erase(bestReqId);
    }
}