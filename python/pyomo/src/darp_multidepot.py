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

        # --- Conjuntos ---
        m.P = pyo.Set(initialize=data['P']) # Pickups
        m.D = pyo.Set(initialize=data['D']) # Deliveries
        m.K = pyo.Set(initialize=data['K']) # Vehículos
        
        # Nodos especiales por vehículo
        start_nodes = list(data['Start'].values())
        end_nodes = list(data['End'].values())
        
        # Todos los nodos relevantes
        m.V = pyo.Set(initialize=data['P'] + data['D'] + start_nodes + end_nodes)
        
        # Arcos válidos: (i,j,k)
        valid_arcs = []
        for k in m.K:
            sk = data['Start'][k]
            ek = data['End'][k]
            nodes_k = data['P'] + data['D'] + [sk, ek]
            for i in nodes_k:
                for j in nodes_k:
                    if i == j: continue
                    if i == ek: continue # No sale del end
                    if j == sk: continue # No entra al start
                    valid_arcs.append((i, j, k))
                    
        m.A_k = pyo.Set(initialize=valid_arcs, dimen=3)

        # --- Parámetros ---
        m.n = pyo.Param(initialize=data['N'])
        m.Q = pyo.Param(m.K, initialize=data['Q'])
        m.L = pyo.Param(initialize=data['L'])
        m.T_max = pyo.Param(m.K, initialize=data['T_max'])
        
        # Costos y Tiempos
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
        m.r_time = pyo.Var(m.P, m.K, domain=pyo.NonNegativeReals)

        # --- Función Objetivo ---
        def obj_rule(m):
            return sum(m.c[i, j, k] * m.x[i, j, k] for (i, j, k) in m.A_k)
        m.Obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        # --- Restricciones ---

        # 1. Cada petición servida una vez
        def serve_request_rule(m, i):
            # FIX: Usamos ii, j, kk del iterador
            return sum(m.x[ii, j, kk] for (ii, j, kk) in m.A_k if ii == i) == 1
        m.ServeRequest = pyo.Constraint(m.P, rule=serve_request_rule)

        # 2. Conservación de flujo
        def flow_balance_rule(m, i, k):
            if i not in data['P'] and i not in data['D']:
                return pyo.Constraint.Skip
            # FIX: Usamos los iteradores correctos del set
            in_flow = sum(m.x[jj, ii, kk] for (jj, ii, kk) in m.A_k if ii == i and kk == k)
            out_flow = sum(m.x[ii, jj, kk] for (ii, jj, kk) in m.A_k if ii == i and kk == k)
            return in_flow - out_flow == 0
        m.FlowBalance = pyo.Constraint((m.P | m.D), m.K, rule=flow_balance_rule)

        # 3. Flujo en Depósitos
        def start_depot_rule(m, k):
            sk = data['Start'][k]
            return sum(m.x[sk, j, k] for (ii, j, kk) in m.A_k if ii == sk and kk == k) == 1
        m.StartDepot = pyo.Constraint(m.K, rule=start_depot_rule)

        def end_depot_rule(m, k):
            ek = data['End'][k]
            return sum(m.x[j, ek, k] for (j, ii, kk) in m.A_k if ii == ek and kk == k) == 1
        m.EndDepot = pyo.Constraint(m.K, rule=end_depot_rule)

        # 4. Pairing
        def pairing_rule(m, i, k):
            delivery_node = i + m.n.value
            sum_i = sum(m.x[i, j, k] for (ii, j, kk) in m.A_k if ii == i and kk == k)
            sum_delivery = sum(m.x[delivery_node, j, k] for (ii, j, kk) in m.A_k if ii == delivery_node and kk == k)
            return sum_i - sum_delivery == 0
        m.Pairing = pyo.Constraint(m.P, m.K, rule=pairing_rule)

        # 5. Tiempos
        def time_consistency_rule(m, i, j, k):
            M = 10000 
            return m.u[j, k] >= m.u[i, k] + m.serv_time[i] + m.travel_time[i, j] - M * (1 - m.x[i, j, k])
        m.TimeConsistency = pyo.Constraint(m.A_k, rule=time_consistency_rule)

        # 6. Ride Time y Precedencia
        def ride_time_calc_rule(m, i, k):
            delivery_node = i + m.n.value
            return m.r_time[i, k] == m.u[delivery_node, k] - (m.u[i, k] + m.serv_time[i])
        m.RideTimeCalc = pyo.Constraint(m.P, m.K, rule=ride_time_calc_rule)

        def ride_time_limit_rule(m, i, k):
            return m.r_time[i, k] <= m.L
        m.RideTimeLimit = pyo.Constraint(m.P, m.K, rule=ride_time_limit_rule)
        
        def precedence_rule(m, i, k):
            delivery_node = i + m.n.value
            return m.u[delivery_node, k] >= m.u[i, k] + m.serv_time[i] + m.travel_time[i, delivery_node]
        m.Precedence = pyo.Constraint(m.P, m.K, rule=precedence_rule)

        # 7. Carga
        def load_consistency_rule(m, i, j, k):
            M = 1000 
            return m.w[j, k] >= m.w[i, k] + m.load_q[j] - M * (1 - m.x[i, j, k])
        m.LoadConsistency = pyo.Constraint(m.A_k, rule=load_consistency_rule)

        def capacity_limit_rule(m, i, k):
            return m.w[i, k] <= m.Q[k]
        m.CapacityLimit = pyo.Constraint(m.V, m.K, rule=capacity_limit_rule)

        # 8. Ventanas de Tiempo
        def time_window_rule_lower(m, i, k):
            return m.early[i] <= m.u[i, k]
        m.TimeWindowLower = pyo.Constraint(m.V, m.K, rule=time_window_rule_lower)
        
        def time_window_rule_upper(m, i, k):
            return m.u[i, k] <= m.late[i]
        m.TimeWindowUpper = pyo.Constraint(m.V, m.K, rule=time_window_rule_upper)

    def solve(self, solver_name='cplex', executable_path=None, time_limit=60):
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
            
            # Construir mapa de siguientes nodos para este vehículo k
            next_node_map = {}
            for (i, j, kk) in self.model.A_k:
                if kk == k and pyo.value(self.model.x[i, j, k]) > 0.5:
                    next_node_map[i] = j
            
            # Iniciar ruta desde el Start Node específico de k
            current_node = self.data['Start'][k]
            end_node = self.data['End'][k]
            
            # Si el vehículo no sale del depósito, saltar
            if current_node not in next_node_map:
                continue
                
            while True:
                if current_node not in next_node_map:
                    break
                    
                next_n = next_node_map[current_node]
                
                # Extraer datos de variables
                start_service = pyo.value(self.model.u[current_node, k])
                service_dur = self.model.serv_time[current_node]
                departure = start_service + service_dur
                
                # Arrival al siguiente
                arrival_next = pyo.value(self.model.u[next_n, k])
                
                # Carga al salir del nodo actual (o llegar al siguiente)
                # Nota: w[i,k] es carga al salir de i.
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
                    # Guardamos la info final del end node (llegada) si se quiere, 
                    # pero el bucle se rompe aquí.
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

        print("\n--- RESUMEN DE RUTAS ---")
        for k, legs in summary.items():
            if not legs:
                print(f"Vehículo {k} no utilizado.")
                continue
                
            start_node = legs[0]['from_node']
            print(f"Vehículo {k} (Start: {start_node}):")
            
            for leg in legs:
                print(f"  {leg['from_node']:<4} -> {leg['to_node']:<4} | "
                      f"Arr: {leg['arrival_time']:5.1f} | "  # Arrival at destination
                      f"ServStart: {leg['start_service_time']:5.1f} | "
                      f"Load: {leg['load_after']:<3.1f}")
            print("")