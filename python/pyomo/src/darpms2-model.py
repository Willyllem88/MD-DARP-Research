import pyomo.environ as pyo

def create_pdp_model(data):
    """
    Crea el modelo Pyomo para el Pickup & Delivery Problem con Multi-Depot.
    
    data: Diccionario con:
        - 'P': Lista de nodos pickup [1, ..., n]
        - 'D': Lista de nodos delivery [n+1, ..., 2n]
        - 'K': Lista de vehículos [1, ..., m]
        - 'Start': Diccionario {k: start_node_id}
        - 'End': Diccionario {k: end_node_id}
        - 'N': Total de peticiones (n)
        - 'd': Diccionario de duración de servicio {i: duration}
        - 'q': Diccionario de demanda {i: load} (positivo para P, negativo para D)
        - 'e', 'l': Ventanas de tiempo {i: time}
        - 'Q': Capacidad del vehículo {k: capacity}
        - 'T_max': Tiempo máximo de ruta {k: time}
        - 'L': Tiempo máximo de viaje por pasajero (Ride time)
        - 'c': Costos de viaje {(i,j,k): cost}
        - 't': Tiempos de viaje {(i,j): time}
    """
    m = pyo.ConcreteModel()

    # --- Conjuntos ---
    m.P = pyo.Set(initialize=data['P']) # Pickups
    m.D = pyo.Set(initialize=data['D']) # Deliveries
    m.K = pyo.Set(initialize=data['K']) # Vehículos
    
    # Nodos especiales por vehículo (Multi-Depot)
    # Asumimos que los nodos start/end están en el grafo global pero son únicos por k
    start_nodes = list(data['Start'].values())
    end_nodes = list(data['End'].values())
    
    # Todos los nodos relevantes: P U D U Starts U Ends
    m.V = pyo.Set(initialize=data['P'] + data['D'] + start_nodes + end_nodes)
    
    # Arcos válidos: (i,j) para cada k. 
    # Simplificación: creamos set de arcos A_k para cada vehículo
    # Un vehículo k va de Start_k -> {P} -> {D} -> End_k
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
    # c[i,j,k]
    m.c = pyo.Param(m.A_k, initialize=lambda m, i, j, k: data['c'].get((i,j,k), 1000))
    # t[i,j] (asumimos igual para todos los k, o indexar por k si es necesario)
    m.travel_time = pyo.Param(m.V, m.V, initialize=lambda m, i, j: data['t'].get((i,j), 0), default=0)
    
    # Service time, Load, Time Windows
    m.serv_time = pyo.Param(m.V, initialize=data['d'], default=0)
    m.load_q = pyo.Param(m.V, initialize=data['q'], default=0)
    m.early = pyo.Param(m.V, initialize=data['e'], default=0)
    m.late = pyo.Param(m.V, initialize=data['l'], default=10000)

    # --- Variables ---
    # x[i, j, k] = 1 si el vehículo k viaja de i a j
    m.x = pyo.Var(m.A_k, domain=pyo.Binary)
    
    # u[i, k] = Tiempo de inicio de servicio en nodo i por vehiculo k
    # Definido para nodos visitables por k.
    m.u = pyo.Var(m.V, m.K, domain=pyo.NonNegativeReals)
    
    # w[i, k] = Carga del vehículo k al salir del nodo i
    m.w = pyo.Var(m.V, m.K, domain=pyo.NonNegativeReals)
    
    # r[i, k] = Tiempo de viaje del usuario i (ride time) - Solo para Pickups
    m.r_time = pyo.Var(m.P, m.K, domain=pyo.NonNegativeReals)

    # --- Función Objetivo (Minimizar Costos) ---
    def obj_rule(m):
        return sum(m.c[i, j, k] * m.x[i, j, k] for (i, j, k) in m.A_k)
    m.Obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

    # --- Restricciones ---

    # 1. Cada petición (Pickup) es servida exactamente una vez
    def serve_request_rule(m, i):
        # Suma sobre todos los k que pueden servir i y todos los j
        return sum(m.x[i, j, k] for (ii, j, kk) in m.A_k if ii == i) == 1
    m.ServeRequest = pyo.Constraint(m.P, rule=serve_request_rule)

    # 2. Flujo en Pickups y Deliveries (Conservación de flujo)
    def flow_balance_rule(m, i, k):
        # Entra = Sale para nodos P U D
        if i not in data['P'] and i not in data['D']:
            return pyo.Constraint.Skip
        
        in_flow = sum(m.x[j, i, k] for (jj, ii, kk) in m.A_k if ii == i and kk == k)
        out_flow = sum(m.x[i, j, k] for (ii, jj, kk) in m.A_k if ii == i and kk == k)
        return in_flow - out_flow == 0
    m.FlowBalance = pyo.Constraint((m.P | m.D), m.K, rule=flow_balance_rule)

    # 3. Flujo en Depósitos (Multi-Depot Extension)
    def start_depot_rule(m, k):
        sk = data['Start'][k]
        # Debe salir exactamente 1 vez de su depot de inicio
        return sum(m.x[sk, j, k] for (ii, j, kk) in m.A_k if ii == sk and kk == k) == 1
    m.StartDepot = pyo.Constraint(m.K, rule=start_depot_rule)

    def end_depot_rule(m, k):
        ek = data['End'][k]
        # Debe entrar exactamente 1 vez a su depot final
        return sum(m.x[j, ek, k] for (j, ii, kk) in m.A_k if ii == ek and kk == k) == 1
    m.EndDepot = pyo.Constraint(m.K, rule=end_depot_rule)

    # 4. Pairing: Si vehículo k visita i, debe visitar n+i
    def pairing_rule(m, i, k):
        delivery_node = i + m.n.value
        # Suma de salidas de i en k - Suma de salidas de n+i en k = 0
        sum_i = sum(m.x[i, j, k] for (ii, j, kk) in m.A_k if ii == i and kk == k)
        sum_delivery = sum(m.x[delivery_node, j, k] for (ii, j, kk) in m.A_k if ii == delivery_node and kk == k)
        return sum_i - sum_delivery == 0
    m.Pairing = pyo.Constraint(m.P, m.K, rule=pairing_rule)

    # 5. Tiempos (Big M method linearizado)
    # u_j >= u_i + d_i + t_ij  si x_ijk = 1
    def time_consistency_rule(m, i, j, k):
        M = 10000 # Ajustar según escala del problema
        return m.u[j, k] >= m.u[i, k] + m.serv_time[i] + m.travel_time[i, j] - M * (1 - m.x[i, j, k])
    m.TimeConsistency = pyo.Constraint(m.A_k, rule=time_consistency_rule)

    # 6. Precedencia y Ride Time (L)
    # n+i ocurre después de i, y la diferencia es <= L
    def ride_time_calc_rule(m, i, k):
        delivery_node = i + m.n.value
        # r[i,k] = u[n+i] - (u[i] + d[i])
        return m.r_time[i, k] == m.u[delivery_node, k] - (m.u[i, k] + m.serv_time[i])
    m.RideTimeCalc = pyo.Constraint(m.P, m.K, rule=ride_time_calc_rule)

    def ride_time_limit_rule(m, i, k):
        return m.r_time[i, k] <= m.L
    m.RideTimeLimit = pyo.Constraint(m.P, m.K, rule=ride_time_limit_rule)
    
    def precedence_rule(m, i, k):
        # u[n+i] >= u[i] + d[i] + t_i,n+i
        delivery_node = i + m.n.value
        return m.u[delivery_node, k] >= m.u[i, k] + m.serv_time[i] + m.travel_time[i, delivery_node]
    m.Precedence = pyo.Constraint(m.P, m.K, rule=precedence_rule)

    # 7. Carga (Capacity)
    def load_consistency_rule(m, i, j, k):
        M = 1000 # Big M
        # w[j] >= w[i] + q[j] si x=1
        return m.w[j, k] >= m.w[i, k] + m.load_q[j] - M * (1 - m.x[i, j, k])
    m.LoadConsistency = pyo.Constraint(m.A_k, rule=load_consistency_rule)

    def capacity_limit_rule(m, i, k):
        # w[i,k] <= Q[k]
        # También w[i,k] >= max(0, q[i])
        return m.w[i, k] <= m.Q[k]
    m.CapacityLimit = pyo.Constraint(m.V, m.K, rule=capacity_limit_rule)

    # 8. Ventanas de Tiempo
    def time_window_rule_lower(m, i, k):
        return m.early[i] <= m.u[i, k]
    m.TimeWindowLower = pyo.Constraint(m.V, m.K, rule=time_window_rule_lower)
    
    def time_window_rule_upper(m, i, k):
        return m.u[i, k] <= m.late[i]
    m.TimeWindowUpper = pyo.Constraint(m.V, m.K, rule=time_window_rule_upper)

    return m