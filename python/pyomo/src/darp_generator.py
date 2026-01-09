import osmnx as ox
import networkx as nx
import random
import json
import os

class DARPInstanceGenerator:
    def __init__(self, place_name, n_requests, n_vehicles):
        self.place_name = place_name
        self.n_requests = n_requests
        self.n_vehicles = n_vehicles
        self.G = None
        self.data = {}
        
    def download_graph(self):
        print(f"🌍 Descargando grafo de '{self.place_name}'...")
        # Descargar red de conducción
        self.G = ox.graph_from_place(self.place_name, network_type='drive')
        
        # Añadir velocidades y tiempos
        self.G = ox.add_edge_speeds(self.G)
        self.G = ox.add_edge_travel_times(self.G)
        
        # Quedarse con el componente fuertemente conectado más grande (evitar islas)
        largest_scc = max(nx.strongly_connected_components(self.G), key=len)
        self.G = self.G.subgraph(largest_scc).copy()
        
        print(f"✅ Grafo cargado con {len(self.G.nodes())} nodos.")

    def generate_instance(self):
        if self.G is None:
            self.download_graph()
            
        all_nodes = list(self.G.nodes())
        
        # Puntos totales: Pickups + Deliveries + StartNodes + EndNodes
        total_points = 2 * self.n_requests + 2 * self.n_vehicles
        
        if len(all_nodes) < total_points:
            raise ValueError("El mapa es muy pequeño para tantos puntos.")

        # Selección aleatoria
        selected_nodes = random.sample(all_nodes, total_points)
        
        # Separar en listas lógicas
        pickups = selected_nodes[:self.n_requests]
        deliveries = selected_nodes[self.n_requests : 2*self.n_requests]
        depots = selected_nodes[2*self.n_requests:]
        
        # Mapeos
        logical_ids = {} # 1 -> OSM_ID
        osm_ids = {}     # OSM_ID -> 1
        
        # 1. Requests (1..N pickups, N+1..2N deliveries)
        current_idx = 1
        for p in pickups:
            logical_ids[current_idx] = p; osm_ids[p] = current_idx; current_idx += 1
        for d in deliveries:
            logical_ids[current_idx] = d; osm_ids[d] = current_idx; current_idx += 1
            
        # 2. Vehicles
        vehicle_configs = []
        depot_map = {} # "d1s" -> OSM_ID
        
        for k in range(1, self.n_vehicles + 1):
            idx_s = (k-1)*2
            idx_e = (k-1)*2 + 1
            
            s_id, e_id = f"d{k}s", f"d{k}e"
            osm_s, osm_e = depots[idx_s], depots[idx_e]
            
            depot_map[s_id] = osm_s
            depot_map[e_id] = osm_e
            
            vehicle_configs.append({
                "id": k, "start_node": s_id, "end_node": e_id,
                "capacity": 3, "max_time": 6000
            })
            
        # 3. Calcular Matrices (Dijkstra)
        print("🧮 Calculando matriz de tiempos (Dijkstra)...")
        
        all_logical_keys = list(logical_ids.keys()) + list(depot_map.keys())
        
        # Helper para obtener OSM node
        def get_osm(lid):
            return logical_ids[lid] if isinstance(lid, int) else depot_map[lid]
        
        matrix_t = []
        matrix_c = []
        
        for origin in all_logical_keys:
            for dest in all_logical_keys:
                if origin == dest: continue
                
                u, v = get_osm(origin), get_osm(dest)
                try:
                    length = nx.shortest_path_length(self.G, u, v, weight='travel_time')
                    val = int(round(length))
                    
                    matrix_t.append({"from": origin, "to": dest, "value": val})
                    # Coste idéntico para todos los vehículos
                    for k in range(1, self.n_vehicles+1):
                        matrix_c.append({"k": k, "from": origin, "to": dest, "value": val})
                        
                except nx.NetworkXNoPath:
                    pass # Si no hay ruta, no añadimos arco (el solver lo tratará como infactible)

        # 4. Estructurar JSON
        self.data = {
            "solver_options": { "solver_name": "cplex", "time_limit": 60 },
            "global_params": { "L_ride": 1800 },
            "requests": {
                "n_requests": self.n_requests,
                "pickup_ids": list(range(1, self.n_requests+1)),
                "delivery_ids": list(range(self.n_requests+1, 2*self.n_requests+1))
            },
            "vehicles": vehicle_configs,
            "nodes": [],
            "matrix_t": matrix_t,
            "matrix_c": matrix_c,
            "metadata": {
                "city": self.place_name,
                "coordinates": {}
            }
        }
        
        # Llenar Metadata y Nodos
        # Pickups/Deliveries
        for i in range(1, self.n_requests + 1):
            # Pickup
            osm_p = logical_ids[i]
            self.data["nodes"].append({"id": i, "service_time": 60, "demand": 1, "tw_start": 0, "tw_end": 10000})
            self.data["metadata"]["coordinates"][str(i)] = (self.G.nodes[osm_p]['y'], self.G.nodes[osm_p]['x'])
            
            # Delivery
            osm_d = logical_ids[i + self.n_requests]
            j = i + self.n_requests
            self.data["nodes"].append({"id": j, "service_time": 60, "demand": -1, "tw_start": 0, "tw_end": 10000})
            self.data["metadata"]["coordinates"][str(j)] = (self.G.nodes[osm_d]['y'], self.G.nodes[osm_d]['x'])
            
        # Depots
        for k_id, osm_n in depot_map.items():
            self.data["nodes"].append({"id": k_id, "service_time": 0, "demand": 0, "tw_start": 0, "tw_end": 10000})
            self.data["metadata"]["coordinates"][k_id] = (self.G.nodes[osm_n]['y'], self.G.nodes[osm_n]['x'])

        return self.data

    def save_json(self, filename="darp_instance.json"):
        with open(filename, 'w') as f:
            json.dump(self.data, f, indent=2)
        print(f"💾 JSON guardado en: {filename}")

# Bloque para probar la clase directamente
if __name__ == "__main__":
    generator = DARPInstanceGenerator("Gracia, Barcelona, Spain", n_requests=10, n_vehicles=2)
    generator.generate_instance()
    generator.save_json("barcelona_darp.json")