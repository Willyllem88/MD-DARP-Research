from darp_multidepot import DARPMultiDepot

def get_example_data():
    """
    Genera los datos del ejemplo pequeño (5 requests, 2 vehículos).
    """
    n_requests = 5
    P = list(range(1, n_requests + 1))           # 1..5
    D = list(range(n_requests + 1, 2*n_requests + 1))  # 6..10
    K = [1, 2]

    # Depósitos diferenciados
    Start = { 1: "d1s", 2: "d2s" }
    End =   { 1: "d1e", 2: "d2e" }

    V = P + D + list(Start.values()) + list(End.values())

    # Service time
    d = {i: 1 for i in V}

    # Demand: +1 pickup, -1 delivery, 0 depósitos
    q = {}
    for i in P:
        q[i] = 1
        q[i + n_requests] = -1
    for v in Start.values(): q[v] = 0
    for v in End.values():   q[v] = 0

    # Time windows
    e = {i: 0 for i in V}
    l = {i: 100 for i in V}

    # Capacities & Max Times
    Q_cap = { 1: 3, 2: 2 }
    T_max = { 1: 100, 2: 100 }
    L_ride = 30

    # Travel times & Costs
    t = {}
    for i in V:
        for j in V:
            if i != j:
                t[(i, j)] = 5
    
    c = {}
    for k in K:
        for i in V:
            for j in V:
                if i != j:
                    c[(i, j, k)] = 1

    data = {
        'P': P, 'D': D, 'K': K,
        'Start': Start, 'End': End,
        'N': n_requests,
        'd': d, 'q': q,
        'e': e, 'l': l,
        'Q': Q_cap,
        'T_max': T_max,
        'L': L_ride,
        'c': c, 't': t
    }
    return data

if __name__ == "__main__":
    # 1. Obtener datos
    data = get_example_data()
    
    # 2. Instanciar el modelo (Estilo Clase)
    darp = DARPMultiDepot(data)
    
    # 3. Resolver
    try:
        # Asegúrate de tener cplex instalado o cambia a 'glpk'/'cbc'
        darp.solve(solver_name='cplex', time_limit=60)
        
        # 4. Imprimir resultados
        darp.print_route_summary()
        
    except Exception as e:
        print(f"Ocurrió un error: {e}")