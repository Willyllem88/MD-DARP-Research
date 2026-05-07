#include "SetPartitioningSolver.h"
#include "ALNSEvaluator.h"
#include <ilcplex/ilocplex.h>
#include <vector>
#include <algorithm>

// Constructor: Initializes the persistent environment and pre-calculates indices
SetPartitioningSolver::SetPartitioningSolver(const DARPMD_ProblemInstance& data, 
                                             const ALNSParams& params, 
                                             ALNSEvaluator& evaluator,
                                             Logger& logger)
    : SetBasedSolver(data, params, evaluator, logger) {
    // Pre-calculate requestToIndex mapping for O(1) access during column generation
    int idx = 0;
    for(int i : data.P) {
        requestToIndex[i] = idx++;
    }
}

bool SetPartitioningSolver::solve(ALNSSolution& newSol, double maxTime) {
    std::unordered_map<int, std::vector<ALNSRoute>> routePool = getRoutePool().getRoutes();
    
    // If no routes are available, return an empty solution
    if (routePool.empty()) {
        return false;
    }

    // Create Model and Cplex objects locally linked to the persistent Env
    IloModel model(env);
    IloCplex cplex(model);

    // CPLEX Configuration
    cplex.setOut(env.getNullStream()); // Silence output
    cplex.setParam(IloCplex::Param::TimeLimit, maxTime);
    cplex.setParam(IloCplex::Param::Threads, 1); // Single thread usually faster for SPP subproblems

    try {
        // --- 1. Define Objectives and Constraints Skeleton ---
        
        // Objective Function: Minimize total cost
        IloObjective obj = IloMinimize(env);
        model.add(obj);

        // Constraint (a): Each request covered exactly once (Partitioning)
        // Stored in an array for fast indexing during column generation
        IloRangeArray requestConstraints(env, data.P.size(), 1.0, 1.0);
        model.add(requestConstraints);

        // Constraint (b): Each vehicle used exactly once
        // Map Vehicle ID -> Constraint Index
        std::unordered_map<int, int> vehicleToIndex;
        int vIdx = 0;
        for (auto const& [k, routes] : routePool) {
            vehicleToIndex[k] = vIdx++;
        }
        IloRangeArray vehicleConstraints(env, vehicleToIndex.size(), 1.0, 1.0);
        model.add(vehicleConstraints);

        // --- 2. Create Variables (Column Generation) ---
        
        // Array to store all decision variables
        IloNumVarArray vars(env);
        
        // Metadata to map decision variables back to routes/requests
        struct VarInfo { 
            int vehicleId; 
            int routeIdx; 
        };
        std::vector<VarInfo> varMetadata;
        // Optimization: Reserve memory to avoid reallocations
        size_t estimatedVars = routePool.size() * 20 + data.P.size(); 
        varMetadata.reserve(estimatedVars); 

        // A. Route Variables (y_rk)
        // Iterate through the pool and create a column for each route
        for (auto const& [k, routes] : routePool) {
            int currentVehicleRowIdx = vehicleToIndex[k];

            for (size_t rIdx = 0; rIdx < routes.size(); ++rIdx) {
                const auto& route = routes[rIdx];
                
                // Initialize column with Objective Coefficient
                IloNumColumn col = obj(route.totalCost);
                
                // Add coefficient for Vehicle Constraint
                col += vehicleConstraints[currentVehicleRowIdx](1.0);

                // Add coefficients for Request Constraints (Requests visited by this route)
                for (int nodeId : route.sequence) {
                    auto it = requestToIndex.find(nodeId);
                    if (it != requestToIndex.end()) {
                        int rowIdx = it->second;
                        col += requestConstraints[rowIdx](1.0);
                    }
                }

                // Create the binary variable using the constructed column
                vars.add(IloNumVar(col, 0.0, 1.0, ILOBOOL));
                varMetadata.push_back({k, (int)rIdx});
                
                // Important: Release the column object to prevent memory bloat
                col.end(); 
            }
        }

        // --- 3. Solve and Reconstruct ---
        
        if (cplex.solve()) {
            IloNumArray vals(env);
            cplex.getValues(vals, vars);
            
            // Clear previous data
            newSol.routes.clear();
            newSol.unassignedRequests.clear();

            // Iterate over solution values
            for (int i = 0; i < vals.getSize(); ++i) {
                // Check if variable is selected ( > 0.5 for binary tolerance)
                if (vals[i] > 0.5) { 
                    const auto& info = varMetadata[i];
                    newSol.routes.push_back(routePool.at(info.vehicleId)[info.routeIdx]);
                }
            }
            vals.end(); // Release value array

            // Final evaluation of the assembled solution
            evaluator.evaluateSolution(newSol);

            // Logs informativos del proceso de CPLEX (Status)
            int totalRoutes = 0;
            for (const auto& [k, routes] : routePool) {
                totalRoutes += routes.size();
            }
            logger.log("  [SetPartitioning] Total Routes in Pool: " + std::to_string(totalRoutes));
            logger.log("  [SetPartitioning] CPLEX Status: " + std::string(
                cplex.getStatus() == IloAlgorithm::Infeasible ? "Infeasible" : 
                cplex.getStatus() == IloAlgorithm::Optimal ? "Optimal" :
                cplex.getStatus() == IloAlgorithm::Feasible ? "Feasible (Time Limit)" : "Unknown"));
            logger.log("  [SetPartitioning] CPLEX time: " + std::to_string(cplex.getTime()) + " seconds");
        }
        else {
            logger.log("  [SetPartitioning] CPLEX found no solution. Status: " + std::to_string(cplex.getStatus()));
            return false;
        }

        // --- 4. Critical Memory Cleanup ---
        // Since 'env' is persistent, we MUST manually destroy the modeling objects
        // created in this scope to avoid memory leaks over multiple iterations.
        
        vars.endElements();        // Destroys all IloNumVar objects
        vars.end();                // Destroys the array container
        
        requestConstraints.endElements(); // Destroys all IloRange objects for requests
        vehicleConstraints.endElements(); // Destroys all IloRange objects for vehicles
        
        obj.end();                 // Destroys the Objective object

    } catch (IloException& e) {
        std::cerr << "[SetPartitioningSolver] CPLEX Exception: " << e << std::endl;
        return false;
    }

    // 'model' and 'cplex' are destroyed here automatically (stack allocated),
    // but the objects inside 'env' were cleaned up above.
    return true;
}