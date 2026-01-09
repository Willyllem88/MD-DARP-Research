import osmnx as ox
import networkx as nx
import json

class DARPGraphManager:
    def __init__(self):
        self.G = None
        self.place_name = ""
        self.nodes_data = {}  # {node_id: {'y': lat, 'x': lon}}
    
    def download_graph(self, place_name):
        """Descarga el grafo, limpia y se queda con la SCC más grande."""
        self.place_name = place_name
        print(f"🌍 Descargando grafo de '{place_name}'...")
        
        # Descargar red de conducción
        self.G = ox.graph_from_place(place_name, network_type='drive')
        
        # Añadir velocidades y tiempos
        self.G = ox.add_edge_speeds(self.G)
        self.G = ox.add_edge_travel_times(self.G)
        
        # Quedarse con el componente fuertemente conectado más grande (SCC)
        largest_scc = max(nx.strongly_connected_components(self.G), key=len)
        self.G = self.G.subgraph(largest_scc).copy()
        
        # Cachear datos de nodos para acceso rápido
        for node, data in self.G.nodes(data=True):
            self.nodes_data[node] = {'y': data['y'], 'x': data['x']}
            
        print(f"✅ Grafo listo con {len(self.G.nodes())} nodos.")
        return self.G

    def get_nearest_node(self, lat, lon):
        """Encuentra el nodo del grafo más cercano a un click (lat, lon)."""
        if self.G is None:
            return None
        return ox.nearest_nodes(self.G, lon, lat) # osmnx usa (X, Y) -> (lon, lat)

    def generate_json_structure(self, user_requests, user_vehicles, global_params):
        """
        Calcula matrices y genera el JSON final.
        user_requests: lista de dicts con info de pickups y deliveries.
        user_vehicles: lista de dicts con info de vehículos.
        """
        print("🧮 Calculando rutas más cortas (Dijkstra) y generando JSON...")
        
        # 1. Recopilar todos los nodos relevantes (Pickups, Deliveries, Depots)
        # Mapeo: Logical ID (1, 2...) -> OSM Node ID
        logical_ids = {}
        osm_ids_map = {} # OSM -> Logical (para debug)
        
        # Estructura de nodos para el JSON
        json_nodes = []
        coordinates = {}
        
        # --- PROCESAR REQUESTS ---
        # IDs lógicos: Pickups 1..N, Deliveries N+1..2N
        n_requests = len(user_requests)
        
        for i, req in enumerate(user_requests):
            req_id = i + 1 # 1-based index
            
            # PICKUP
            p_logical_id = req_id
            p_osm = req['pickup_node']
            logical_ids[p_logical_id] = p_osm
            
            json_nodes.append({
                "id": p_logical_id,
                "service_time": req['p_service_time'],
                "demand": req['demand'],
                "tw_start": req['p_tw_start'],
                "tw_end": req['p_tw_end']
            })
            coordinates[str(p_logical_id)] = (self.nodes_data[p_osm]['y'], self.nodes_data[p_osm]['x'])
            
            # DELIVERY
            d_logical_id = req_id + n_requests
            d_osm = req['delivery_node']
            logical_ids[d_logical_id] = d_osm
            
            json_nodes.append({
                "id": d_logical_id,
                "service_time": req['d_service_time'],
                "demand": -req['demand'], # Demanda negativa
                "tw_start": req['d_tw_start'],
                "tw_end": req['d_tw_end']
            })
            coordinates[str(d_logical_id)] = (self.nodes_data[d_osm]['y'], self.nodes_data[d_osm]['x'])

        # --- PROCESAR VEHICULOS ---
        vehicle_configs = []
        depot_map = {} # "d1s" -> OSM_ID
        
        for k_idx, veh in enumerate(user_vehicles):
            k = k_idx + 1
            s_id, e_id = f"d{k}s", f"d{k}e"
            
            osm_s = veh['start_node']
            osm_e = veh['end_node']
            
            depot_map[s_id] = osm_s
            depot_map[e_id] = osm_e
            
            # Config del vehículo
            vehicle_configs.append({
                "id": k, 
                "start_node": s_id, 
                "end_node": e_id,
                "capacity": veh['capacity'], 
                "max_time": veh['max_time']
            })
            
            # Añadir nodos depot al JSON de nodos
            # Start
            json_nodes.append({"id": s_id, "service_time": 0, "demand": 0, "tw_start": 0, "tw_end": 100000})
            coordinates[s_id] = (self.nodes_data[osm_s]['y'], self.nodes_data[osm_s]['x'])
            # End
            json_nodes.append({"id": e_id, "service_time": 0, "demand": 0, "tw_start": 0, "tw_end": 100000})
            coordinates[e_id] = (self.nodes_data[osm_e]['y'], self.nodes_data[osm_e]['x'])

        # --- CALCULO DE MATRICES ---
        # Unimos todas las claves (IDs numéricos de requests + IDs string de depots)
        all_keys = list(logical_ids.keys()) + list(depot_map.keys())
        
        def get_osm(lid):
            if lid in logical_ids: return logical_ids[lid]
            return depot_map[lid]
            
        matrix_t = []
        matrix_c = []
        
        for origin in all_keys:
            for dest in all_keys:
                if origin == dest: continue
                
                u = get_osm(origin)
                v = get_osm(dest)
                
                try:
                    # shortest_path_length de NetworkX con peso 'travel_time'
                    length = nx.shortest_path_length(self.G, u, v, weight='travel_time')
                    val = int(round(length))
                    
                    matrix_t.append({"from": origin, "to": dest, "value": val})
                    
                    # Coste igual al tiempo para todos los vehículos (según ejemplo)
                    for k in range(1, len(user_vehicles) + 1):
                        matrix_c.append({"k": k, "from": origin, "to": dest, "value": val})
                        
                except nx.NetworkXNoPath:
                    pass # Si no hay camino en la SCC (raro pero posible), se omite

        # --- ENSAMBLAR FINAL ---
        data = {
            "solver_options": { "solver_name": "cplex", "time_limit": 60 },
            "global_params": global_params,
            "requests": {
                "n_requests": n_requests,
                "pickup_ids": list(range(1, n_requests + 1)),
                "delivery_ids": list(range(n_requests + 1, 2 * n_requests + 1))
            },
            "vehicles": vehicle_configs,
            "nodes": json_nodes,
            "matrix_t": matrix_t,
            "matrix_c": matrix_c,
            "metadata": {
                "city": self.place_name,
                "coordinates": coordinates
            }
        }
        
        return data

    def save_to_file(self, data, filename="output_darp.json"):
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"💾 Guardado en {filename}")