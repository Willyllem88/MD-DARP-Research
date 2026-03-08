import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import pyomo.environ as pyo
from pyomo.opt import SolverStatus, TerminationCondition

from MDDARP_ProblemInstance import MDDARP_ProblemInstance

# --- Result classes ---

@dataclass
class RouteStep:
    node_id: int = 0
    type: str = ""
    arrival_time: float = 0.0
    load_after: float = 0.0

@dataclass
class VehicleRoute:
    vehicle_id: int = 0
    steps: List[RouteStep] = field(default_factory=list)

@dataclass
class MDDARP_ResultInstance:
    objective_value: float = 0.0
    solve_time: float = 0.0
    mip_gap: float = 0.0
    solver_status: str = "Unknown"
    routes: Dict[int, VehicleRoute] = field(default_factory=dict)

    def add_route(self, k: int, route: VehicleRoute):
        self.routes[k] = route

    def display(self):
        print("\n============================================")
        print("              SOLUTION RESULTS              ")
        print("============================================")
        print(f"Status:      {self.solver_status}")
        print(f"Objective:   {self.objective_value:.4f}")
        print(f"Solve Time:  {self.solve_time:.4f} s")
        print(f"MIP Gap:     {self.mip_gap * 100:.4f}%")
        print("--------------------------------------------")
        for k, route in self.routes.items():
            print(f"Vehicle {k} Route:")
            for step in route.steps:
                print(f"  -> Node {step.node_id:>3} [{step.type:^12}] | Time: {step.arrival_time:>6.2f} | Load: {step.load_after:>5.2f}")
        print("============================================\n")



class MDDARP_Model_Solver:
    def __init__(self, instance, time_limit: Optional[float] = None, solver_name: Optional[str] = 'cplex'):
        self.data: MDDARP_ProblemInstance = instance
        self.time_limit: Optional[float] = time_limit
        self.solver_name: Optional[str] = solver_name if solver_name else 'cplex'

        self.model = pyo.ConcreteModel()
        
        # Result attributes
        self.solve_time: float = 0.0
        self.objective_value: float = 0.0
        self.mip_gap: float = 0.0
        self.solver_status_str: str = "Not Solved"

        # 1. Create Set V: P u D u StartNodes u EndNodes
        distinct_nodes = set(self.data.P + self.data.D)
        for node in self.data.StartNodes: distinct_nodes.add(node)
        for node in self.data.EndNodes: distinct_nodes.add(node)
        self.V_nodes: List[int] = list(distinct_nodes)

        # 2. Crear Set A_k: Arcos válidos
        self.A_k = []
        for k in self.data.K:
            sk = self.data.start_node[k]
            ek = self.data.end_node[k]
            nodes_k = self.data.P + self.data.D + [sk, ek]
            
            for i in nodes_k:
                for j in nodes_k:
                    if i == j: continue
                    if i == ek: continue  # Do not leave End depot
                    if j == sk: continue  # Do not begin before Start depot
                    if (self.data.is_delivery(i)) and (i == j + self.data.N_requests): continue      # Do not go from delivery_i to its pickup_i
                    if (self.data.is_vehicle_start(i)) and (self.data.is_delivery(j)): continue      # Do not go from Start depot to delivery
                    if (self.data.is_pickup(i)) and (self.data.is_vehicle_end(j)): continue          # Do not go from pickup to End depot

                    self.A_k.append((i, j, k))

        print(f"Total arcs in A_k: {len(self.A_k)}")

        self.build_model()

    def build_model(self):
        print("Creating the Pyomo model...")
        m = self.model
        d = self.data

        # --- 1. Variables ---
        m.x = pyo.Var(self.A_k, domain=pyo.Binary)
        m.u = pyo.Var(self.V_nodes, domain=pyo.NonNegativeReals)
        
        w_indices = [(i, k) for i in self.V_nodes for k in d.K]
        m.w = pyo.Var(w_indices, domain=pyo.NonNegativeReals)

        # --- 2. Objective Function ---
        def obj_rule(model):
            return sum(d.get_cost(i, j, k) * model.x[i, j, k] for (i, j, k) in self.A_k)
        m.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        # --- 3. Constraints ---
        m.constraints = pyo.ConstraintList()

        # c1: Each request served once
        for i in d.P:
            m.constraints.add(
                sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if ii == i) == 1
            )

        # c2: Flow conservation
        PuD = d.P + d.D
        for i in PuD:
            for k in d.K:
                in_flow = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if kk == k and jj == i)
                out_flow = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if kk == k and ii == i)
                m.constraints.add(in_flow - out_flow == 0)

        # c3: Flow at Depots
        for k in d.K:
            sk = d.start_node[k]
            ek = d.end_node[k]
            start_expr = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if ii == sk and kk == k)
            end_expr = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if jj == ek and kk == k)
            m.constraints.add(start_expr == 1)
            m.constraints.add(end_expr == 1)

        # c4: Pairing
        for i in d.P:
            delivery_node = i + d.N_requests
            for k in d.K:
                flow_pick = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if kk == k and ii == i)
                flow_del = sum(m.x[ii, jj, kk] for (ii, jj, kk) in self.A_k if kk == k and ii == delivery_node)
                m.constraints.add(flow_pick - flow_del == 0)

        # c5: Time Consistency
        for (i, j, k) in self.A_k:
            serv = d.get_service_time(i)
            trav = d.get_travel_time(i, j)
            l_i = d.get_time_window_end(i)
            e_j = d.get_time_window_start(j)
            
            M_ij = max(0.0, l_i + serv + trav - e_j)
            m.constraints.add(m.u[j] >= m.u[i] + serv + trav - M_ij * (1 - m.x[i, j, k]))

        # c6: Time Windows
        for i in self.V_nodes:
            ei = d.get_time_window_start(i)
            li = d.get_time_window_end(i)

            m.constraints.add(m.u[i] >= ei)
            m.constraints.add(m.u[i] <= li)

        # c7: Max Route Duration
        for k in d.K:
            sk = d.start_node[k]
            ek = d.end_node[k]
            Tmax = d.get_vehicle_max_route_time(k)
            m.constraints.add(m.u[ek] - m.u[sk] <= Tmax)

        # c8: Precedence (Pickup before Delivery)
        for i in d.P:
            delivery_node = i + d.N_requests
            serv = d.get_service_time(i)
            trav = d.get_travel_time(i, delivery_node)
            m.constraints.add(m.u[delivery_node] >= m.u[i] + serv + trav)

        # c9: Ride Time Limit
        for i in d.P:
            delivery_node = i + d.N_requests
            serv = d.get_service_time(i)
            L = d.max_ride_time
            m.constraints.add(m.u[delivery_node] - (m.u[i] + serv) <= L)

        # c10: Load Consistency
        for (i, j, k) in self.A_k:
            q_j = d.get_demand(j)
            Q_k = d.get_vehicle_capacity(k)
            M_ijk = Q_k
            m.constraints.add(m.w[j, k] >= m.w[i, k] + q_j - M_ijk * (1 - m.x[i, j, k]))

        # c11: Load Feasible Bounds
        for i in self.V_nodes:
            qi = d.get_demand(i)
            for k in d.K:
                Qk = d.get_vehicle_capacity(k)
                lb = max(0.0, qi)
                ub = min(Qk, Qk + qi)
                m.constraints.add(m.w[i, k] >= lb)
                m.constraints.add(m.w[i, k] <= ub)

        # c12: Depot Initial/Final Load
        for k in d.K:
            sk = d.start_node[k]
            ek = d.end_node[k]
            m.constraints.add(m.w[sk, k] == 0)
            m.constraints.add(m.w[ek, k] == 0)

    def solve(self):
        print(f"Starting resolution with {self.solver_name.upper()}...")
        
        solver = pyo.SolverFactory(self.solver_name)
        
        if self.time_limit is not None:
            solver.options['timelimit'] = self.time_limit

        start_time = time.perf_counter()
        results = solver.solve(self.model, tee=True) # tee=True to show solver output in console
        end_time = time.perf_counter()

        self.solve_time = end_time - start_time

        if (results.solver.status == SolverStatus.ok) and \
           (results.solver.termination_condition == TerminationCondition.optimal):
            self.solver_status_str = "Optimal"
            self.objective_value = pyo.value(self.model.obj)
            # Extract the Gap in Pyomo requires reading the attributes of the solver
            try:
                # Depending on the solver, the attributes might be different. This is a generic way to try to get the gap.
                lb = results.problem.lower_bound
                ub = results.problem.upper_bound
                if ub > 0:
                    self.mip_gap = abs(ub - lb) / ub
            except:
                self.mip_gap = 0.0

            print(f"Status: Optimal")
            print(f"Objective Value: {self.objective_value}")
        
        elif results.solver.termination_condition == TerminationCondition.feasible:
            self.solver_status_str = "Feasible"
            self.objective_value = pyo.value(self.model.obj)
            print(f"Status: Feasible (Time limit reached?)")
            
        else:
            self.solver_status_str = str(results.solver.termination_condition)
            print(f"No solution found. Status: {self.solver_status_str}")

        print(f"Total Solve Time: {self.solve_time:.2f} s")

    def get_result(self) -> MDDARP_ResultInstance:
        result = MDDARP_ResultInstance()
        result.solve_time = self.solve_time
        result.solver_status = self.solver_status_str
        
        if self.solver_status_str not in ["Optimal", "Feasible"]:
            return result

        result.objective_value = self.objective_value
        result.mip_gap = self.mip_gap

        # Reconstruct routes
        for k in self.data.K:
            v_route = VehicleRoute(vehicle_id=k)
            next_node_map = {}
            
            # Search for active arcs for vehicle k
            for (i, j, veh) in self.A_k:
                if veh == k:
                    # Numerical tolerance for binary variables
                    if pyo.value(self.model.x[i, j, k]) > 0.5:
                        next_node_map[i] = j

            current_node = self.data.start_node[k]
            end_node = self.data.end_node[k]

            # If the vehicle does not move (no next node for the start_node)
            if current_node not in next_node_map:
                v_route.steps.append(RouteStep(node_id=current_node, type="DepotStart"))
                result.add_route(k, v_route)
                continue

            # Traverse the route
            while True:
                step = RouteStep(node_id=current_node)
                
                # Determine type
                if current_node == self.data.start_node[k]: step.type = "DepotStart"
                elif current_node == self.data.end_node[k]: step.type = "DepotEnd"
                elif current_node in self.data.P: step.type = "Pickup"
                elif current_node in self.data.D: step.type = "Delivery"
                else: step.type = "Node"

                # Extract u (time) and w (load)
                step.arrival_time = pyo.value(self.model.u[current_node])
                
                if (current_node, k) in self.model.w:
                    step.load_after = pyo.value(self.model.w[current_node, k])

                v_route.steps.append(step)

                # Stop condition
                if current_node == end_node: break
                if current_node not in next_node_map: break

                # Advance
                current_node = next_node_map[current_node]

            result.add_route(k, v_route)

        return result


# --- Main for testing ---

if __name__ == "__main__":    
    
    json_path = "/home/guillem/TFG-Guillem/data/instances_static/cordeau-instances/a3-24.json"
    
    instance = MDDARP_ProblemInstance()    
    success = instance.load_from_json(json_path)
    
    if success:
        instance.display_info()
        
        solver = MDDARP_Model_Solver(instance, time_limit=60.0, solver_name='cplex')
        solver.solve()
        
        results = solver.get_result()
        results.display()
    else:
        print("Failed to load instance from JSON.")