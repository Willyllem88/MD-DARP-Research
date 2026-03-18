import time
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import pyomo.environ as pyo
from pyomo.opt import SolverStatus, TerminationCondition

from MDDARP_ProblemInstance import MDDARP_ProblemInstance

VERBOSE = True


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

def clamp(x, lower, upper):
    if x < lower: return lower
    if x > upper: return upper
    return x

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

        # 3. Apply Time-Window Tightening
        self._tighten_time_windows()

        # 2. Create Set A_k: Valid arcs
        self.A_k = []
        for k in self.data.K:
            sk = self.data.start_node[k]
            ek = self.data.end_node[k]
            nodes_k = self.data.P + self.data.D + [sk, ek]
            
            for i in nodes_k:
                for j in nodes_k:
                    if self._is_arc_feasible(i, j, k):
                        self.A_k.append((i, j, k))

        nb_a_k_nodes: int = len(self.A_k)
        total_posible_arcs: int = len(self.V_nodes) * len(self.V_nodes) * len(self.data.K)
        ratio_pruned: float = ((total_posible_arcs - nb_a_k_nodes) / total_posible_arcs) * 100
        print(f"Total arcs generated (A_k): {nb_a_k_nodes}")
        print(f"Total possible arcs: {total_posible_arcs}")
        print(f"Ratio pruned: {ratio_pruned:.2f}%\n")

        self.build_model()

    def _tighten_time_windows(self):
        """
        Apply the Time-Window Tightening (Cordeau 2006) to reduce the width of
        the time windows without eliminating optimal solutions.
        """
        d = self.data
        L = d.max_ride_time
        N = d.N_requests
        
        # T: End of the planning horizon. 
        # We estimate it as the maximum upper limit between all final depots.
        T = max([d.get_time_window_end(ek) for ek in d.EndNodes])

        # --- 1. Pickup and Delivery adjustment ---
        for i in d.P:
            del_i = i + N
            
            # Original values
            e_i = d.get_time_window_start(i)
            l_i = d.get_time_window_end(i)
            e_n_i = d.get_time_window_start(del_i)
            l_n_i = d.get_time_window_end(del_i)
            
            serv_i = d.get_service_time(i)
            t_i_ni = d.get_travel_time(i, del_i)
            
            # Formulas of tightening for the Origin (Outbound user context)
            new_e_i = max(e_i, e_n_i - L - serv_i)
            new_l_i = min(l_i, l_n_i - t_i_ni - serv_i, T)
            
            # Formulas of tightening for the Destination (Inbound user context)
            new_e_n_i = max(e_n_i, e_i + serv_i + t_i_ni)
            new_l_n_i = min(l_n_i, l_i + serv_i + L, T)
            
            # Update the instance of data with the new tightened time windows
            d.update_time_window(i, new_e_i, new_l_i)
            d.update_time_window(del_i, new_e_n_i, new_l_n_i)
            
        # --- 2. Depots adjustment ---
        for k in d.K:
            sk = d.start_node[k]
            ek = d.end_node[k]
            PuD = d.P + d.D
            
            # Original values
            e_sk = d.get_time_window_start(sk)
            l_sk = d.get_time_window_end(sk)
            e_ek = d.get_time_window_start(ek)
            l_ek = d.get_time_window_end(ek)
            
            # --- 1. Theoretical bounds (The '*' variables) ---
            
            # Start depot: Earliest and latest possible departures dictated by customers
            e_star_sk = min([d.get_time_window_start(j) - d.get_travel_time(sk, j) for j in d.P])
            l_star_sk = max([d.get_time_window_end(j)   - d.get_travel_time(sk, j) for j in d.P])
            
            # End depot: Earliest and latest possible arrivals dictated by customers
            e_star_ek = min([d.get_time_window_start(j) + d.get_service_time(j) + d.get_travel_time(j, ek) for j in d.D])
            l_star_ek = max([d.get_time_window_end(j)   + d.get_service_time(j) + d.get_travel_time(j, ek) for j in d.D])
            
            # --- 2. Bounding / Clamping to ensure feasibility ---
            
            # Start depot adjustment
            new_e_sk = clamp(e_star_sk, e_sk, l_sk)
            new_l_sk = clamp(l_star_sk, new_e_sk, l_sk)
            
            # End depot adjustment (Remember: Calculate latest FIRST)
            new_l_ek = clamp(l_star_ek, e_ek, l_ek)
            new_e_ek = clamp(e_star_ek, e_ek, new_l_ek)
            
            # Update the instance of data with the fully tightened time windows
            d.update_time_window(sk, new_e_sk, new_l_sk)
            d.update_time_window(ek, new_e_ek, new_l_ek)

    def _is_arc_feasible(self, i: int, j: int, k: int) -> bool:
        """
        Evaluates whether arc (i, j) is feasible for vehicle k based on
        precedence rules, time windows, capacity, and maximum ride time.
        Returns True if the arc is valid, False if it should be pruned.
        """
        d = self.data
        sk = d.start_node[k]
        ek = d.end_node[k]
        N = d.N_requests
        epsilon = 1e-4

        # --- 1. Basic logical and structural rules ---
        if i == j: return False
        if i == ek: return False  # Cannot leave the end depot
        if j == sk: return False  # Cannot arrive at the start depot
        if d.is_delivery(i) and i == j + N: return False  # Cannot go from a delivery to its own pickup
        if d.is_vehicle_start(i) and d.is_delivery(j): return False  # Cannot go from start depot directly to a delivery
        if d.is_pickup(i) and d.is_vehicle_end(j): return False  # Cannot go from a pickup directly to the end depot

        ei = d.get_time_window_start(i)
        di = d.get_service_time(i)
        t_ij = d.get_travel_time(i, j)
        lj = d.get_time_window_end(j)

        # --- 2. Feasible return to the end depot ---
        if j != ek:
            Ej_start = max(ei + di + t_ij, d.get_time_window_start(j))
            dj = d.get_service_time(j)
            l_ek = d.get_time_window_end(ek)

            if d.is_pickup(j):
                # If 'j' is a pickup, its delivery must be served before going to the depot
                delivery_j = j + N
                d_del_j = d.get_service_time(delivery_j)
                t_j_del = d.get_travel_time(j, delivery_j)
                t_delj_ek = d.get_travel_time(delivery_j, ek)

                e_del_j_start = max(Ej_start + dj + t_j_del, d.get_time_window_start(delivery_j))
                if e_del_j_start + d_del_j + t_delj_ek > l_ek + epsilon:
                    return False
            else:
                # If 'j' is a delivery (or start depot), check direct return to end depot
                t_j_ek = d.get_travel_time(j, ek)
                if Ej_start + dj + t_j_ek > l_ek + epsilon:
                    return False

            
        # --- 3. Cross validation ---

        # Case 3a: Two consecutive pickups (i is pickup, j is pickup)
        if d.is_pickup(i) and d.is_pickup(j):
            del_i = i + N
            del_j = j + N
            # If we pick up both, we have to deliver them. There are two possible orders.
            # If NONE of the two orders is feasible, then the arc (i, j) is impossible.
            if not (self.check_path_feasibility([i, j, del_i, del_j], k) or 
                    self.check_path_feasibility([i, j, del_j, del_i], k)):
                return False

        # Case 3b: Two consecutive deliveries (i is delivery, j is delivery)
        elif d.is_delivery(i) and d.is_delivery(j):
            pick_i = i - N
            pick_j = j - N
            # If we deliver to both, we had to pick them up before. There are two possible orders.
            if not (self.check_path_feasibility([pick_i, pick_j, i, j], k) or 
                    self.check_path_feasibility([pick_j, pick_i, i, j], k)):
                return False

        # Case 3c: Pickup followed by delivery of another passenger (i is pickup, j is delivery)
        elif d.is_pickup(i) and d.is_delivery(j) and j != i + N:
            del_i = i + N
            pick_j = j - N
            # To do this, we must have picked up 'j' BEFORE picking up 'i',
            # and we will deliver 'i' AFTER delivering 'j'. This path is unique and linear.
            if not self.check_path_feasibility([pick_j, i, j, del_i], k):
                return False

        # Case 3d: Delivery followed by pickup of another passenger (i is delivery, j is pickup)
        elif d.is_delivery(i) and d.is_pickup(j):
            pick_i = i - N
            del_j = j + N
            # To do this, we must have picked up 'i' BEFORE delivering 'i',
            # and we will deliver 'j' AFTER picking up 'j'. This path is unique and linear.
            if not self.check_path_feasibility([pick_i, i, j, del_j], k):
                return False

        return True
    
    def check_path_feasibility(self, path: List[int], k: int) -> bool:
        """
        Evaluates if a specific sequence of nodes (path) is feasible.
        It is 100% safe: it only returns False if it is mathematically impossible to satisfy 
        the requirements, avoiding pruning optimal solutions.
        """
        if len(path) < 2:
            return True

        d = self.data
        epsilon = 1e-4

        # --- 1. CAPACIDAD (Cota inferior estricta) ---
        # Calculamos la variación neta de carga en la secuencia.
        # Si en algún momento la subida neta supera la capacidad máxima del vehículo, es infactible.
        current_load = 0
        max_load_seen = 0
        for node in path:
            current_load += d.get_demand(node)
            if current_load > max_load_seen + epsilon:
                max_load_seen = current_load
                
        if max_load_seen > d.get_vehicle_capacity(k) + epsilon:
            return False

        # --- 2. VENTANAS DE TIEMPO (Llegada más temprana posible) ---
        # Si simulamos el viaje lo más rápido posible y aún así llegamos tarde, el arco está roto.
        t = d.get_time_window_start(path[0])
        for idx in range(1, len(path)):
            prev_n = path[idx - 1]
            curr_n = path[idx]
            
            # Hora de llegada al nodo curr_n
            arrival = max(t, d.get_time_window_start(prev_n)) + d.get_service_time(prev_n) + d.get_travel_time(prev_n, curr_n)
            
            if arrival > d.get_time_window_end(curr_n) + epsilon:
                return False
                
            t = arrival # Actualizamos el reloj para la siguiente iteración

        # --- 3. RIDE TIME (Cota inferior de duración) ---
        # Calculamos el tiempo mínimo absoluto entre un Pickup y su Delivery dentro de la ruta.
        for i in range(len(path)):
            p_node = path[i]
            
            if d.is_pickup(p_node):
                d_node = p_node + d.N_requests
                
                # Si la ruta contiene tanto la recogida como su entrega
                if d_node in path[i+1:]:
                    d_idx = path.index(d_node)
                    
                    # 3a. Sumamos tiempos incompresibles: viajes y servicios intermedios
                    min_ride_time = 0.0
                    for j in range(i, d_idx):
                        min_ride_time += d.get_travel_time(path[j], path[j+1])
                        # El servicio del pickup inicial no cuenta para el ride time (empieza a contar al salir)
                        if j > i: 
                            min_ride_time += d.get_service_time(path[j])
                            
                    # 3b. Sumamos la espera mínima obligatoria (Solo para el nodo inmediatamente posterior)
                    # ¿Qué pasa si salimos del pickup en el último segundo posible (l_i)?
                    if i + 1 < d_idx:
                        next_n = path[i+1]
                        l_p = d.get_time_window_end(p_node)
                        s_p = d.get_service_time(p_node)
                        t_p_next = d.get_travel_time(p_node, next_n)
                        e_next = d.get_time_window_start(next_n)
                        
                        mandatory_wait = max(0.0, e_next - (l_p + s_p + t_p_next))
                        min_ride_time += mandatory_wait

                    if min_ride_time > d.max_ride_time + epsilon:
                        return False

        return True

    def node_alias(self, node_id: int) -> str:
        if self.data.is_vehicle_start(node_id):
            return f"V{node_id - 2*self.data.N_requests}+"
        elif self.data.is_vehicle_end(node_id):
            return f"V{node_id - 2*self.data.N_requests - self.data.K_vehicles}-"
        elif self.data.is_pickup(node_id):
            return f"{node_id}+"
        elif self.data.is_delivery(node_id):
            return f"{node_id - self.data.N_requests}-"
        else:
            return f"N{node_id}"

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
            valid_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if ii == i]
            if not valid_arcs:
                print(f"⚠️ Warning: The pickup node {i} has no valid arcs. The problem is infeasible.")
                m.constraints.add(pyo.Constraint.Infeasible)
            else:
                m.constraints.add(sum(m.x[a] for a in valid_arcs) == 1)

        # c2: Flow conservation
        PuD = d.P + d.D
        for i in PuD:
            for k in d.K:
                in_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if kk == k and jj == i]
                out_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if kk == k and ii == i]

                if not in_arcs and not out_arcs:
                    print(f"⚠️ Warning: The node {i} for the vehicle {k} has no valid arcs. The problem is infeasible.")
                    m.constraints.add(pyo.Constraint.Infeasible)
                    continue

                in_flow = sum(m.x[a] for a in in_arcs) if in_arcs else 0
                out_flow = sum(m.x[a] for a in out_arcs) if out_arcs else 0
                m.constraints.add(in_flow - out_flow == 0)

        # c3: Flow at Depots
        for k in d.K:
            sk = d.start_node[k]
            ek = d.end_node[k]
            start_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if ii == sk and kk == k]
            end_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if jj == ek and kk == k]
            
            if start_arcs: 
                m.constraints.add(sum(m.x[a] for a in start_arcs) == 1)
            else: 
                print(f"⚠️ Warning: The initial depot {sk} for the vehicle {k} has no valid outgoing arcs. The problem is infeasible.")
                m.constraints.add(pyo.Constraint.Infeasible)
                
            if end_arcs: 
                m.constraints.add(sum(m.x[a] for a in end_arcs) == 1)
            else:
                print(f"⚠️ Warning: The final depot {ek} for the vehicle {k} has no valid incoming arcs. The problem is infeasible.")
                m.constraints.add(pyo.Constraint.Infeasible)

        # c4: Pairing
        for i in d.P:
            delivery_node = i + d.N_requests
            for k in d.K:
                pick_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if kk == k and ii == i]
                del_arcs = [(ii, jj, kk) for (ii, jj, kk) in self.A_k if kk == k and ii == delivery_node]
                
                # Si el vehículo 'k' no puede ni recoger ni entregar, evitamos el 0 == 0
                if not pick_arcs and not del_arcs:
                    print(f"⚠️ Alerta: El pedido {i} no tiene arcos válidos para el vehículo {k}. El problema es infactible.")
                    continue
                    
                flow_pick = sum(m.x[a] for a in pick_arcs) if pick_arcs else 0
                flow_del = sum(m.x[a] for a in del_arcs) if del_arcs else 0
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
        print(f"Number of variables: {len(self.model.x) + len(self.model.u) + len(self.model.w)}")
        print(f"Number of constraints: {len(self.model.constraints)}")
        
        solver = pyo.SolverFactory(self.solver_name)
        
        if self.time_limit is not None:
            solver.options['timelimit'] = self.time_limit

        start_time = time.perf_counter()
        results = solver.solve(self.model, tee=VERBOSE)
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

    json_path = "/home/guillem/TFG-Guillem/data/instances_static/cordeau-instances/a4-48.json"
    
    instance = MDDARP_ProblemInstance()    
    success = instance.load_from_json(json_path)
    
    if success:
        instance.display_info()
        
        solver = MDDARP_Model_Solver(instance, solver_name='cplex')
        solver.solve()
        
        results = solver.get_result()
        #results.display()
    else:
        print("Failed to load instance from JSON.")