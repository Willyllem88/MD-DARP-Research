#pragma once

#include "ALNSRoute.h"
#include "../MDDARP_ResultInstance.h"

#include <vector> 
#include <set>
#include <iostream>

// Internal representation of a full solution
struct ALNSSolution {
    std::vector<ALNSRoute> routes;      // One per vehicle
    std::set<int> unassignedRequests;   // IDs of requests in P not served
    double objectiveValue = 0.0;        // Total penalized cost
    bool hasViolations = false;         // Whether the solution has any constraint violations

    // Mapping from node ID to route index for quick access
    std::vector<int> node2routeIndex;

    // Initialize the node-to-route mapping
    void initNodeDirectory(int maxNodeId) {
        node2routeIndex.assign(maxNodeId + 1, -1);
    }

    // Update the node-to-route mapping based on the current routes
    void updateNodeToRouteMapping() {
        for (size_t r = 0; r < routes.size(); ++r) {
            for (int nodeId : routes[r].sequence) {
                node2routeIndex[nodeId] = r;
            }
        }
    }

    inline int getRouteIndexOf(int nodeId) const { return node2routeIndex[nodeId]; }
    inline const ALNSRoute* getRouteOf(int nodeId) const {
        int rIdx = node2routeIndex[nodeId];
        if (rIdx == -1) return nullptr; // Nodo no asignado
        return &routes[rIdx];
    }

    inline double getA(int nodeId) const {
        const ALNSRoute* r = getRouteOf(nodeId);
        return r ? r->getA(nodeId) : -1.0;
    }

    inline double getB(int nodeId) const {
        const ALNSRoute* r = getRouteOf(nodeId);
        return r ? r->getB(nodeId) : -1.0;
    }

    inline double getW(int nodeId) const {
        const ALNSRoute* r = getRouteOf(nodeId);
        return r ? r->getW(nodeId) : -1.0;
    }

    inline double getD(int nodeId) const {
        const ALNSRoute* r = getRouteOf(nodeId);
        return r ? r->getD(nodeId) : -1.0;
    }

    inline double getLoad(int nodeId) const {
        const ALNSRoute* r = getRouteOf(nodeId);
        return r ? r->getLoad(nodeId) : -1.0;
    }


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

    MDDARP_ResultInstance toResultInstance(const MDDARP_ProblemInstance& problem, double computationTime = 0.0) const {
        // Initialize the result instance with the problem
        MDDARP_ResultInstance result(problem);
        
        // Map the metadata to the solution
        result.objectiveValue = this->objectiveValue;
        result.solveTime = computationTime;
        result.mipGap = 0.0;
        
        if (this->hasViolations) {
            result.solverStatus = "Semi-Feasible";
        } else {
            result.solverStatus = "Feasible";
        }

        int numReq = problem.N_requests;
        int numVeh = problem.K_vehicles;

        // Translate each route
        for (const auto& alnsRoute : this->routes) {
            VehicleRoute vr;
            vr.vehicleId = alnsRoute.vehicleId;
            vr.routeCost = alnsRoute.distanceCost;
            
            vr.routeDuration = alnsRoute.getAByPos(alnsRoute.getRouteSize() - 1) - alnsRoute.getDByPos(0); // Arrival at end depot - Departure from start depot

            // Translate each stop in the sequence to a RouteStep
            for (size_t i = 0; i < alnsRoute.sequence.size(); ++i) {
                RouteStep step;
                step.nodeId = alnsRoute.sequence[i];
                
                step.beginServiceTime = alnsRoute.getBByPos(i);
                step.loadAfter = alnsRoute.getLoadByPos(i);

                // Determine the type of node based on its ID
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

            // Add the translated route to the result instance
            result.addRoute(vr.vehicleId, vr);
        }

        // Calculate the formal violations using the instance's own logic
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