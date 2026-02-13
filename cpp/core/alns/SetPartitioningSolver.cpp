#include "SetPartitioningSolver.h"
#include "ALNSEvaluator.h"
#include <ilcplex/ilocplex.h>

SetPartitioningSolver::SetPartitioningSolver(const DARPMD_ProblemInstance& data, 
                                             const ALNSParams& params, 
                                             ALNSEvaluator& evaluator) 
    : data(data), params(params), evaluator(evaluator) {} 

ALNSSolution SetPartitioningSolver::solve(const std::map<int, std::vector<ALNSRoute>>& routePool) const {
    ALNSSolution newSol;
    
    // Si no hay rutas, devolvemos una solución vacía/inválida
    if (routePool.empty()) {
        return newSol; 
    }

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
        size_t totalRoutes = 0;
        for (auto const& [k, routes] : routePool) {
            totalRoutes += routes.size();
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
            for (size_t rIdx = 0; rIdx < routePool.at(k).size(); ++rIdx) {
                vehicleUsage += y[{k, rIdx}];
            }
            model.add(vehicleUsage <= 1);
            vehicleUsage.end();
        }

        // 3. Solve
        spCplex.setParam(IloCplex::Param::TimeLimit, 10.0); // Strict limit for SP
        
        if (spCplex.solve()) {
            // Reconstruct Solution
            newSol.unassignedRequests.clear();
            newSol.routes.clear(); // Asegurar limpieza

            // Active Routes
            for (auto const& [key, var] : y) {
                if (spCplex.getValue(var) > 0.5) {
                    int k = key.first;
                    int rIdx = key.second;
                    // Aseguramos acceso seguro con at()
                    newSol.routes.push_back(routePool.at(k)[rIdx]);
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
                    evaluator.evaluateRoute(emptyR);
                    newSol.routes.push_back(emptyR);
                }
            }

            // Unassigned
            for (int i : data.P) {
                if (spCplex.getValue(z[i]) > 0.5) {
                    newSol.unassignedRequests.insert(i);
                }
            }

            // Evaluate complete solution to fill costs/objective
            evaluator.evaluateSolution(newSol);
            
            // Logs informativos del proceso de CPLEX (Status)
            std::cout << "  [SetPartitioning] Total Routes in Pool: " << totalRoutes << std::endl;
            std::cout << "  [SetPartitioning] CPLEX Status: " 
                      << (spCplex.getStatus() == IloAlgorithm::Infeasible ? "Infeasible" : 
                          spCplex.getStatus() == IloAlgorithm::Optimal ? "Optimal" :
                          spCplex.getStatus() == IloAlgorithm::Feasible ? "Feasible (Time Limit)" : "Unknown") << std::endl;
            std::cout << "  [SetPartitioning] CPLEX time: " << spCplex.getTime() << " seconds" << std::endl;
        }

    } catch (IloException& e) {
        std::cerr << "CPLEX Exception in SetPartitioningSolver: " << e << std::endl;
        throw; // Relanzar o manejar según preferencia
    }
    env.end();

    return newSol;
}