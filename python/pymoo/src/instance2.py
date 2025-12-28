import math
from darp_model import DARPModelLB

def calculate_distance(p1, p2):
    return round(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2), 1)

def get_instance_10_requests():
    """
    Genera una instancia DARP con 10 solicitudes (20 nodos + depósito).
    Distribución espacial en rejilla 20x20 y 3 franjas horarias.
    """
    requests = list(range(1, 11))
    pickups = [f"{r}+" for r in requests]
    deliveries = [f"{r}-" for r in requests]
    nodes = pickups + deliveries
    all_nodes = ['0_start'] + nodes + ['0_end']

    # Coordenadas (x, y) - Depósito Central (10, 10)
    coords = {
        '0_start': (10, 10), '0_end': (10, 10),
        '1+': (2, 18),  '1-': (18, 2),   # Diagonal larga
        '2+': (3, 3),   '2-': (5, 5),    # Corto alcance
        '3+': (17, 17), '3-': (15, 12),
        '4+': (1, 5),   '4-': (10, 2),
        '5+': (19, 5),  '5-': (12, 8),
        '6+': (2, 2),   '6-': (8, 8),
        '7+': (15, 3),  '7-': (18, 15),
        '8+': (5, 15),  '8-': (2, 10),
        '9+': (10, 18), '9-': (14, 14),
        '10+': (8, 1),  '10-': (1, 12)
    }

    travel_times = {(i, j): calculate_distance(coords[i], coords[j]) for i in all_nodes for j in all_nodes}

    # Ventanas de tiempo (e_j, l_j) divididas en 3 turnos
    # Turno 1 (Early): Requests 1-4 | Turno 2 (Mid): 5-7 | Turno 3 (Late): 8-10
    tw_data = {
        # 'node': (start, end)
        '0_start': (0, 200), '0_end': (0, 250),
        '1+': (0, 40),   '1-': (10, 80),
        '2+': (5, 45),   '2-': (15, 85),
        '3+': (10, 50),  '3-': (20, 90),
        '4+': (20, 60),  '4-': (30, 100),
        '5+': (60, 100), '5-': (70, 140),
        '6+': (65, 105), '6-': (75, 145),
        '7+': (70, 110), '7-': (80, 150),
        '8+': (110, 150),'8-': (120, 190),
        '9+': (120, 160),'9-': (130, 200),
        '10+':(130, 170),'10-':(140, 210),
    }

    data = {
        'Requests': requests,
        'P': pickups,
        'D': deliveries,
        'costs': travel_times,
        'travel_times': travel_times,
        'service_times': {n: 3 for n in nodes} | {'0_start': 0, '0_end': 0},
        'load_change': {
            **{'0_start': 0, '0_end': 0}, 
            **{f"{r}+": 1 for r in requests}, 
            **{f"{r}-": -1 for r in requests}
        },
        'tw_start': {n: tw_data[n][0] for n in all_nodes},
        'tw_end': {n: tw_data[n][1] for n in all_nodes},
        'max_ride_time': {r: 50 for r in requests}, 
        'capacity': 3,
        'vehicles': 3  # Aumentado a 3 para manejar el volumen y las ventanas
    }
    return data

if __name__ == "__main__":
    print("--- Generando Instancia de 10 Solicitudes ---")
    instance_data = get_instance_10_requests()
    
    darp_instance = DARPModelLB(instance_data)
    
    try:
        # Nota: El solver 'cbc' (Open Source) puede tardar unos minutos con 10 requests.
        # Si tienes acceso a CPLEX o Gurobi, es el momento de usarlos.
        print("Optimizando... (Esto puede tardar según el solver)")
        darp_instance.solve(solver_name='cplex') 
        
        print("\n" + "="*30)
        print("SOLUCIÓN ENCONTRADA")
        print("="*30)
        darp_instance.print_route_summary()
        
    except Exception as e:
        print(f"Error en la ejecución: {e}")