#pragma once

#include "ALNSRoute.h"
#include "../DARPMD_ResultInstance.h"

#include <vector> 
#include <set>
#include <iostream>

// Internal representation of a full solution
struct ALNSSolution {
    std::vector<ALNSRoute> routes; // One per vehicle
    std::set<int> unassignedRequests; // IDs of requests in P not served
    double objectiveValue = 0.0; // Total penalized cost
    bool hasViolations = false; // Whether the solution has any constraint violations

    void print() {
        std::cout << "  Routes:" << std::endl;
        for (const auto& r : routes) {
            std::cout << "    Vehicle " << r.vehicleId << ": ";
            for (int node : r.sequence) {
                std::cout << node << " ";
            }
            std::cout << "| Cost: " << r.totalCost
                    << " | Distance: " << r.distanceCost
                    << " | TW Violation: " << r.timeWindowViolation 
                    << " | Load Violation: " << r.loadViolation 
                    << " | Ride Time Violation: " << r.rideTimeViolation
                    << std::endl;
        }
        std::cout << "  Unassigned Requests: ";
        for (int req : unassignedRequests) {
            std::cout << req << " ";
        }
        std::cout << std::endl;
    };

    DARPMD_ResultInstance toResultInstance(const DARPMD_ProblemInstance& problem, double computationTime = 0.0) const {
        // 1. Inicializar la instancia de resultado con el problema
        DARPMD_ResultInstance result(problem);
        
        // 2. Mapear los metadatos generales de la solución
        result.objectiveValue = this->objectiveValue;
        result.solveTime = computationTime;
        result.mipGap = 0.0; // El MIP gap no aplica a heurísticas como ALNS
        
        if (this->hasViolations) {
            result.solverStatus = "Semi-Feasible";
        } else {
            result.solverStatus = "Feasible";
        }

        int numReq = problem.N_requests;
        int numVeh = problem.K_vehicles;

        // 3. Traducir cada ruta
        for (const auto& alnsRoute : this->routes) {
            VehicleRoute vr;
            vr.vehicleId = alnsRoute.vehicleId;
            vr.routeCost = alnsRoute.distanceCost; // O alnsRoute.totalCost si prefieres el coste penalizado
            
            // Calcular la duración de la ruta (llegada al Depot final - salida al Depot inicial)
            if (!alnsRoute.B.empty()) {
                vr.routeDuration = alnsRoute.A.back() - alnsRoute.D.front();
            } else {
                vr.routeDuration = 0.0;
            }

            // Traducir cada parada en la secuencia a un RouteStep
            for (size_t i = 0; i < alnsRoute.sequence.size(); ++i) {
                RouteStep step;
                step.nodeId = alnsRoute.sequence[i];
                
                // Usar los vectores especificados
                step.beginServiceTime = (i < alnsRoute.B.size()) ? alnsRoute.B[i] : 0.0;
                step.loadAfter = (i < alnsRoute.loads.size()) ? alnsRoute.loads[i] : 0.0;

                // Determinar el tipo de nodo en función de su ID
                if (step.nodeId <= numReq) {
                    step.type = "Pickup";
                } else if (step.nodeId <= 2 * numReq) {
                    step.type = "Delivery";
                } else if (step.nodeId <= 2 * numReq + numVeh) {
                    step.type = "DepotStart";
                } else {
                    step.type = "DepotEnd";
                }

                vr.steps.push_back(step);
            }

            // Añadir la ruta traducida a la instancia de resultados
            result.addRoute(vr.vehicleId, vr);
        }

        // 4. Calcular las violaciones formales utilizando la propia lógica de la instancia
        result.calculateViolations();

        return result;
    }
};

struct SolutionHash {
    std::size_t operator()(const ALNSSolution& sol) const {
        std::size_t hash = 0;
        RouteSequenceHash routeHasher;

        for (const auto& route : sol.routes) {
            hash ^= routeHasher(route.sequence) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        
        return hash;
    }
};