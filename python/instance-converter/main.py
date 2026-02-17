import math
import json
import argparse
import sys

def parse_line(line):
    return [float(x) for x in line.split()]

def read_cordeau_instance(filepath):
    try:
        with open(filepath, 'r') as f:
            lines = [l.strip() for l in f if l.strip()]
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo {filepath}")
        sys.exit(1)

    # 1. Parse Global Parameters
    header = parse_line(lines[0])
    num_vehicles = int(header[0])
    num_requests = int(header[1])
    max_route_time = header[2]
    capacity = int(header[3])
    max_ride_time = header[4]

    # 2. Parse Nodes
    raw_nodes = {}
    for i in range(1, len(lines)):
        row = parse_line(lines[i])
        node_id = int(row[0])
        raw_nodes[node_id] = {
            "id": node_id,
            "x": row[1],
            "y": row[2],
            "service_time": row[3],
            "demand": int(row[4]),
            "tw_start": row[5],
            "tw_end": row[6]
        }

    return {
        "num_vehicles": num_vehicles,
        "num_requests": num_requests,
        "max_route_time": max_route_time,
        "capacity": capacity,
        "max_ride_time": max_ride_time,
        "raw_nodes": raw_nodes
    }

def euclidean_distance(n1, n2):
    return math.sqrt((n1['x'] - n2['x'])**2 + (n1['y'] - n2['y'])**2)

def generate_json_structure(data):
    raw_nodes = data['raw_nodes']
    n_req = data['num_requests']
    n_vehicles = data['num_vehicles']
    
    output = {
        "global_params": {
            "L_ride": data['max_ride_time']
        },
        "requests": {
            "n_requests": n_req,
            "pickup_ids": [],
            "delivery_ids": []
        },
        "vehicles": [],
        "nodes": [],
        "matrix_t": [],
        "matrix_c": [],
        "metadata": {
            "city": "_Cordeau_",
            "coordinates": {}
        }
    }

    # 1. Fill Requests
    for i in range(1, n_req + 1):
        output["requests"]["pickup_ids"].append(i)
        output["requests"]["delivery_ids"].append(i + n_req)

    # 2. Process Customer Nodes (Pickups & Deliveries: 1 to 2n)
    # We store them in a temporary dict to access them easily during matrix generation
    all_nodes_map = {} 

    for i in range(1, (2 * n_req) + 1):
        if i in raw_nodes:
            n = raw_nodes[i]
            node_obj = {
                "id": n["id"],
                "service_time": n["service_time"],
                "demand": n["demand"],
                "tw_start": n["tw_start"],
                "tw_end": n["tw_end"],
                "x": n["x"], "y": n["y"] # Keep coords internally for distance calc
            }
            output["nodes"].append({k:v for k,v in node_obj.items() if k not in ['x', 'y']})
            all_nodes_map[i] = node_obj
            output["metadata"]["coordinates"][str(i)] = [n["x"], n["y"]]

    # 3. Create Start/End Depots per Vehicle & Vehicle Objects
    depot_coords = raw_nodes[0] # Original depot coordinates (node 0)

    for k in range(1, n_vehicles + 1):
        # --- DEFINICIÓN DE IDs SEGÚN TU FORMULA ---
        # Start Node: 2n + k
        start_node_id = (2 * n_req) + k
        
        # End Node: 2n + |K| + k
        end_node_id = (2 * n_req) + n_vehicles + k
        
        # Crear nodo Start
        start_node = {
            "id": start_node_id,
            "service_time": 0,
            "demand": 0,
            "tw_start": raw_nodes[0]['tw_start'],
            "tw_end": raw_nodes[0]['tw_end'],
            "x": depot_coords['x'], "y": depot_coords['y']
        }
        output["nodes"].append({k:v for k,v in start_node.items() if k not in ['x', 'y']})
        all_nodes_map[start_node_id] = start_node

        # Crear nodo End
        end_node = {
            "id": end_node_id,
            "service_time": 0,
            "demand": 0,
            "tw_start": raw_nodes[0]['tw_start'],
            "tw_end": raw_nodes[0]['tw_end'],
            "x": depot_coords['x'], "y": depot_coords['y']
        }
        output["nodes"].append({k:v for k,v in end_node.items() if k not in ['x', 'y']})
        all_nodes_map[end_node_id] = end_node

        # Añadir Vehículo
        output["vehicles"].append({
            "id": k,
            "start_node": start_node_id,
            "end_node": end_node_id,
            "capacity": data['capacity'],
            "max_time": data['max_route_time']
        })

        # Añadir coordenadas de los depósitos a metadata
        output["metadata"]["coordinates"][str(start_node_id)] = [depot_coords['y'], depot_coords['x']]
        output["metadata"]["coordinates"][str(end_node_id)] = [depot_coords['y'], depot_coords['x']]

    # 4. Generate Matrices based on Set Ak definition
    # Ak = (i,j) where i,j in P U D U {sk, ek}
    # i != j, i != ek, j != sk
    
    added_arcs_t = set() # To avoid duplicates in matrix_t

    customer_ids = list(range(1, (2 * n_req) + 1))

    for k in range(1, n_vehicles + 1):
        s_k = (2 * n_req) + k
        e_k = (2 * n_req) + n_vehicles + k
        
        # Nodos válidos para el vehículo k
        valid_nodes = customer_ids + [s_k, e_k]

        for i in valid_nodes:
            for j in valid_nodes:
                # Restricciones del Set Ak
                if i == j: continue       # i != j
                if i == e_k: continue     # i != ek (cannot leave sink)
                if j == s_k: continue     # j != sk (cannot enter source)

                # Calcular distancia
                dist = euclidean_distance(all_nodes_map[i], all_nodes_map[j])
                dist = round(dist, 3)

                # Añadir a matrix_c (específica del vehículo k)
                output["matrix_c"].append({
                    "k": k,
                    "from": i,
                    "to": j,
                    "value": dist
                })

                # Añadir a matrix_t (global)
                # Solo añadimos si no existe ya el arco (i, j).
                # Nota: Los arcos que involucran sk o ek son únicos para ese vehículo,
                # pero los arcos entre clientes (i, j donde i,j <= 2n) se repetirán para cada k.
                if (i, j) not in added_arcs_t:
                    output["matrix_t"].append({
                        "from": i,
                        "to": j,
                        "value": dist
                    })
                    added_arcs_t.add((i, j))

    return output

def silent_print(message, silent):
    if not silent:
        print(message)

def main():
    parser = argparse.ArgumentParser(description="Convert Cordeau DARP instances to Multi-Depot JSON (Rigorous Formulation).")
    parser.add_argument("--load", required=True, help="Path to instance file (.txt/.dat)")
    parser.add_argument("--output", required=True, help="Path to output file (.json)")
    parser.add_argument("--silent", action='store_true', help="Run without printing progress messages")
    
    args = parser.parse_args()

    silent_print(f"Reading instance: {args.load}", args.silent)

    data = read_cordeau_instance(args.load)
    
    silent_print("Generating JSON structure...", args.silent)

    json_data = generate_json_structure(data)
    
    silent_print(f"Writing output to: {args.output}", args.silent)
    
    with open(args.output, 'w') as f:
        json.dump(json_data, f, indent=2)
    
    silent_print("Done!", args.silent)

if __name__ == "__main__":
    main()