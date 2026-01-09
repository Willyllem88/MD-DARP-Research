import pyomo.environ as pyo
from pyomo.opt import SolverFactory, TerminationCondition

class DARPMultiDepot:
    """
    Multi-Depot Pickup & Delivery Problem (PDP) formulation.
    Pyomo model with 3-index variables x[i,j,k].
    """

    def __init__(self, data):
        """
        Initializes the model with input data.
        :param data: Dictionary containing sets, parameters, and configuration.
        """
        self.data = data
        self.model = pyo.ConcreteModel()
        self._build_model()
    
    def _build_model(self):
        """
        Internal method to build the Pyomo model (Variables, Constraints, Objective).
        Logic imported strictly from original functional code.
        """
        m = self.model
        data = self.data

        # --- Sets ---
        m.P = pyo.Set(initialize=data['P']) # Pickups
        m.D = pyo.Set(initialize=data['D']) # Deliveries
        m.K = pyo.Set(initialize=data['K']) # Vehicles
        
        # Special nodes per vehicle
        start_nodes = list(data['Start'].values())
        end_nodes = list(data['End'].values())
        
        # All relevant nodes
        m.V = pyo.Set(initialize=data['P'] + data['D'] + start_nodes + end_nodes)
        
        # Valid arcs: (i,j,k)
        valid_arcs = []
        for k in m.K:
            sk = data['Start'][k]
            ek = data['End'][k]
            nodes_k = data['P'] + data['D'] + [sk, ek]
            for i in nodes_k:
                for j in nodes_k:
                    if i == j: continue
                    if i == ek: continue # Does not leave the end
                    if j == sk: continue # Does not enter the start
                    valid_arcs.append((i, j, k))
                    
        m.A_k = pyo.Set(initialize=valid_arcs, dimen=3)

        # --- Parameters ---
        m.n = pyo.Param(initialize=data['N'])
        m.Q = pyo.Param(m.K, initialize=data['Q'])
        m.L = pyo.Param(initialize=data['L'])
        m.T_max = pyo.Param(m.K, initialize=data['T_max'])
        
        # Costs and Times
        m.c = pyo.Param(m.A_k, initialize=lambda m, i, j, k: data['c'].get((i,j,k), 1000))
        m.travel_time = pyo.Param(m.V, m.V, initialize=lambda m, i, j: data['t'].get((i,j), 0), default=0)
        
        # Service time, Load, Time Windows
        m.serv_time = pyo.Param(m.V, initialize=data['d'], default=0)
        m.load_q = pyo.Param(m.V, initialize=data['q'], default=0)
        m.early = pyo.Param(m.V, initialize=data['e'], default=0)
        m.late = pyo.Param(m.V, initialize=data['l'], default=10000)

        # --- Variables ---
        m.x = pyo.Var(m.A_k, domain=pyo.Binary)
        m.u = pyo.Var(m.V, m.K, domain=pyo.NonNegativeReals)
        m.w = pyo.Var(m.V, m.K, domain=pyo.NonNegativeReals)

        # --- Objective Function ---
        def obj_rule(m):
            return sum(m.c[i, j, k] * m.x[i, j, k] for (i, j, k) in m.A_k)
        m.Obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        # --- Constraints ---

        # Each request served once (c1)
        def serve_request_rule(m, i):
            return sum(m.x[i, j, k] for k in m.K for j in m.V if (i, j, k) in m.A_k) == 1
        m.ServeRequest = pyo.Constraint(m.P, rule=serve_request_rule)

        # Flow conservation (c2)
        def flow_balance_rule(m, i, k):
            if i not in data['P'] and i not in data['D']:
                return pyo.Constraint.Skip
            in_flow = sum(m.x[j, i, k] for j in m.V if (j, i, k) in m.A_k)
            out_flow = sum(m.x[i, j, k] for j in m.V if (i, j, k) in m.A_k)
            return in_flow - out_flow == 0
        m.FlowBalance = pyo.Constraint((m.P | m.D), m.K, rule=flow_balance_rule)

        # Flow at Depots (c3)
        def start_depot_rule(m, k):
            sk = data['Start'][k]
            return sum(m.x[sk, j, k] for (ii, j, kk) in m.A_k if ii == sk and kk == k) == 1
        m.StartDepot = pyo.Constraint(m.K, rule=start_depot_rule)

        def end_depot_rule(m, k):
            ek = data['End'][k]
            return sum(m.x[i, ek, k] for (i, ii, kk) in m.A_k if ii == ek and kk == k) == 1
        m.EndDepot = pyo.Constraint(m.K, rule=end_depot_rule)

        # Pairing (c4)
        def pairing_rule(m, i, k):
            delivery_node = i + m.n.value
            sum_i = sum(m.x[i, j, k] for (ii, j, kk) in m.A_k if ii == i and kk == k)
            sum_delivery = sum(m.x[delivery_node, j, k] for (ii, j, kk) in m.A_k if ii == delivery_node and kk == k)
            return sum_i - sum_delivery == 0
        m.Pairing = pyo.Constraint(m.P, m.K, rule=pairing_rule)

        # Consistent times (c5)
        def time_consistency_rule(m, i, j, k):
            M = 10000 
            return m.u[j, k] >= m.u[i, k] + m.serv_time[i] + m.travel_time[i, j] - M * (1 - m.x[i, j, k])
        m.TimeConsistency = pyo.Constraint(m.A_k, rule=time_consistency_rule)

        # Time Windows (c6)
        def time_window_rule(m, i, k):
            return pyo.inequality(m.early[i], m.u[i, k], m.late[i])
        m.TimeWindow = pyo.Constraint(m.V, m.K, rule=time_window_rule)

        # Maximum Ride Time (c7)
        def max_ride_time_rule(m, k):
            return m.u[data['End'][k], k] - m.u[data['Start'][k], k] <= m.T_max[k]
        m.MaxRideTime = pyo.Constraint(m.K, rule=max_ride_time_rule)

        # Precedence (c8)
        def precedence_rule(m, i, k):
            delivery_node = i + m.n.value
            k_serves_i = sum(m.x[i, j, k] for j in m.V if (i, j, k) in m.A_k)
            M = 10000  
            return m.u[delivery_node, k] >= m.u[i, k] + m.serv_time[i] + \
                m.travel_time[i, delivery_node] - M * (1 - k_serves_i)
        m.Precedence = pyo.Constraint(m.P, m.K, rule=precedence_rule)

        # Ride Time within Limit (c9)
        def ride_time_limit_rule(m, i, k):
            delivery_node = i + m.n.value
            k_serves_i = sum(m.x[i, j, k] for j in m.V if (i, j, k) in m.A_k)
            # TODO: tighten M (maybe m.late)
            M = 10000
            return m.u[delivery_node, k] - (m.u[i, k] + m.serv_time[i]) <= m.L + M * (1 - k_serves_i)
        m.RideTimeLimitRule = pyo.Constraint(m.P, m.K, rule=ride_time_limit_rule)

        # Consistent load (c10)
        def load_consistency_rule(m, i, j, k):
            M = 1000 
            return m.w[j, k] >= m.w[i, k] + m.load_q[j] - M * (1 - m.x[i, j, k])
        m.LoadConsistency = pyo.Constraint(m.A_k, rule=load_consistency_rule)

        # Carga dentro de Límites (c11)
        def load_feasible_bounds(m, i, k):
            lower_bound = max(0, m.load_q[i])
            upper_bound = min(m.Q[k], m.Q[k] + m.load_q[i])
            return pyo.inequality(lower_bound, m.w[i, k], upper_bound)
        m.LoadFeasibleBounds = pyo.Constraint(m.V, m.K, rule=load_feasible_bounds)

        # Carga Inicial y Final en Depósitos (c12)
        def depot_initial_load_rule(m, k):
            sk = data['Start'][k]
            return m.w[sk, k] == 0
        pyo.Constraint(m.K, rule=depot_initial_load_rule)
        
        def depot_final_load_rule(m, k):
            ek = data['End'][k]
            return m.w[ek, k] == 0
        pyo.Constraint(m.K, rule=depot_final_load_rule)

    def solve(self, solver_name='cplex', executable_path=None, time_limit=None):
        """
        Solves the model using the specified solver.
        """
        solver = SolverFactory(solver_name, executable=executable_path)
        if time_limit:
            solver.options["timelimit"] = time_limit
            
        results = solver.solve(self.model, tee=True)
        
        if results.solver.termination_condition == TerminationCondition.optimal:
            print("¡Solución óptima encontrada!")
        elif results.solver.termination_condition == TerminationCondition.infeasible:
            print("El modelo es infactible.")
        else:
            print(f"Estado del solver: {results.solver.termination_condition}")
            
        return results

    def get_route_summary(self):
        """
        Extracts solution results for Multi-Depot structure.
        Iterates over vehicles (K) instead of just one list.
        """
        if not hasattr(self.model, 'x'):
            return "Model not solved yet."
            
        summary = {}
        
        # Iterar sobre cada vehículo
        for k in self.model.K:
            vehicle_legs = []
            
            # Build map of next nodes for this vehicle k
            next_node_map = {}
            for (i, j, kk) in self.model.A_k:
                if kk == k and pyo.value(self.model.x[i, j, k]) > 0.5:
                    next_node_map[i] = j
            
            # Start route from the specific Start Node of k
            current_node = self.data['Start'][k]
            end_node = self.data['End'][k]
            
            # If the vehicle does not leave the depot, skip
            if current_node not in next_node_map:
                continue
                
            while True:
                if current_node not in next_node_map:
                    break
                    
                next_n = next_node_map[current_node]
                
                # Extract variable data
                start_service = pyo.value(self.model.u[current_node, k])
                service_dur = self.model.serv_time[current_node]
                departure = start_service + service_dur
                
                # Arrival at the next
                arrival_next = pyo.value(self.model.u[next_n, k])
                
                # Load after leaving the current node (or arriving at the next)
                # Note: w[i,k] is the load after leaving i.
                load_val = pyo.value(self.model.w[current_node, k])
                
                leg_data = {
                    'from_node': current_node,
                    'to_node': next_n,
                    'start_service_time': start_service,
                    'service_duration': service_dur,
                    'departure_time': departure,
                    'arrival_time': arrival_next,
                    'load_after': load_val
                }
                vehicle_legs.append(leg_data)
                
                current_node = next_n
                if current_node == end_node:
                    # We save the final info of the end node (arrival) if desired, 
                    # but the loop breaks here.
                    break
            
            summary[k] = vehicle_legs
            
        return summary

    def print_route_summary(self):
        """
        Prints the route summary in readable format.
        """
        summary = self.get_route_summary()
        
        if isinstance(summary, str):
            print(summary)
            return

        print("\n--- ROUTES SUMMARY ---")
        for k, legs in summary.items():
            if not legs:
                print(f"Vehicle {k} not used.")
                continue
                
            start_node = legs[0]['from_node']
            print(f"Vehicle {k} (Start: {start_node}):")
            
            for leg in legs:
                i = leg['from_node']
                j = leg['to_node']
                def pprint_node(n):
                    if n in self.data['P']:
                        return f"{n}+"
                    elif n in self.data['D']:
                        return f"{n - len(self.data['P'])}-"
                    else:
                        return str(n)
                from_node = pprint_node(i)
                to_node = pprint_node(j)

                print(f"  {from_node:<4} -> {to_node:<4} | "
                      f"Arr: {leg['arrival_time']:5.1f} | "  # Arrival at destination
                      f"ServStart: {leg['start_service_time']:5.1f} | "
                      f"Load: {leg['load_after']:<3.1f}")
            print("")

    # TODO: lift big Ms