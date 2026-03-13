from yaml import warnings
import osmnx as ox
import networkx as nx
import json
import tkinter as tk
from tkinter import filedialog

class DARPGraphManager:
    def __init__(self):
        self.G = None
        self.place_name = ""
        self.nodes_data = {}  # {node_id: {'y': lat, 'x': lon}}
    
    def download_graph(self, place_name):
        """Downloading the graph, cleaning it, and keeping the largest SCC."""
        self.place_name = place_name
        print(f"🌍 Downloading graph for '{place_name}'...")
        
        # Download driving network
        self.G = ox.graph_from_place(place_name, network_type='drive')
        
        # Add speeds and travel times
        self.G = ox.add_edge_speeds(self.G)
        self.G = ox.add_edge_travel_times(self.G)
        
        # Keep the largest strongly connected component (SCC)
        largest_scc = max(nx.strongly_connected_components(self.G), key=len)
        self.G = self.G.subgraph(largest_scc).copy()
        
        # Cache node data for quick access
        for node, data in self.G.nodes(data=True):
            self.nodes_data[node] = {'y': data['y'], 'x': data['x']}
            
        print(f"✅ Graph ready with {len(self.G.nodes())} nodes.")
        return self.G
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the Haversine distance between two points."""
        from math import radians, cos, sin, asin, sqrt
        
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        R = 6371000  # Radius of Earth in meters
        return c * R
    
    def get_nearest_node_distance(self, lat, lon):
        """Get the distance from a click to the nearest node."""
        if self.G is None:
            return None
        
        target_node = ox.nearest_nodes(self.G, lon, lat)
        node_data = self.G.nodes[target_node]
        node_lat = node_data['y']
        node_lon = node_data['x']
        
        dist_meters = self.haversine_distance(lat, lon, node_lat, node_lon)
        return dist_meters

    def get_nearest_node(self, lat, lon):
        """Find the graph node closest to a click (lat, lon)."""
        if self.G is None:
            return None
        return ox.nearest_nodes(self.G, lon, lat) # osmnx uses (X, Y) -> (lon, lat)
    
    def get_travel_time(self, u, v):
        if self.G is None:
            return None
        try:
            length = nx.dijkstra_path_length(self.G, u, v, weight='travel_time')
            return length
        except nx.NetworkXNoPath:
            # Should not happen in SCC
            print(f"⚠️ No path between {u} and {v}")
            return float('inf')

    def generate_json_structure(self, user_requests, user_vehicles, global_params):
        """
        Calculate matrices and generate the final JSON.
        user_requests: list of dicts with info on pickups and deliveries.
        user_vehicles: list of dicts with info on vehicles.
        """
        print("🧮 Calculating shortest paths (Dijkstra all pairs) and generating JSON...")
        
        # 1. Collect all relevant nodes (Pickups, Deliveries, Depots)
        # Mapping: Logical ID (1, 2...) -> OSM Node ID
        logical_ids = {}
        osm_ids_map = {} # OSM -> Logical (for debugging)
        
        # Node structure for JSON
        json_nodes = []
        coordinates = {}
        
        # --- PROCESS REQUESTS ---
        # Logical IDs: Pickups 1..N, Deliveries N+1..2N
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

        # --- PROCESS VEHICLES ---
        vehicle_configs = []
        depot_map = {} # "d1s" -> OSM_ID
        
        for k_idx, veh in enumerate(user_vehicles):
            k = k_idx + 1
            n_vehicles = len(user_vehicles)
            s_id, e_id = n_requests*2 + k, n_requests*2 + n_vehicles + k
            
            osm_s = veh['start_node']
            osm_e = veh['end_node']
            
            depot_map[s_id] = osm_s
            depot_map[e_id] = osm_e
            
            # Vehicle config
            vehicle_configs.append({
                "id": k, 
                "start_node": s_id, 
                "end_node": e_id,
                "capacity": veh['capacity'], 
                "max_time": veh['max_time']
            })
            
            # Add depot nodes to JSON nodes
            # Start
            json_nodes.append({"id": s_id, "service_time": 0, "demand": 0, "tw_start": veh['vstart_tw_start'], "tw_end": veh['vstart_tw_end']})
            coordinates[s_id] = (self.nodes_data[osm_s]['y'], self.nodes_data[osm_s]['x'])
            # End
            json_nodes.append({"id": e_id, "service_time": 0, "demand": 0, "tw_start": veh['vend_tw_start'], "tw_end": veh['vend_tw_end']})
            coordinates[e_id] = (self.nodes_data[osm_e]['y'], self.nodes_data[osm_e]['x'])

        # --- CALCULATE MATRICES ---
        # Combine all keys (Numeric IDs of requests + String IDs of depots)
        all_keys = list(logical_ids.keys()) + list(depot_map.keys())
        
        def get_osm(lid):
            if lid in logical_ids: return logical_ids[lid]
            return depot_map[lid]
            
        matrix_t = []
        matrix_c = []
        
        # Iterate only over the origin to run Dijkstra once per origin
        for origin in all_keys:
            u = get_osm(origin)
            
            if u not in self.G:
                continue

            # Calculate distances from 'u' to ALL reachable nodes at once
            # This returns a dictionary: {destination_node: distance, ...}
            try:
                lengths = nx.single_source_dijkstra_path_length(self.G, u, weight='travel_time')
            except Exception as e:
                print(f"Error calculating routes from {u}: {e}")
                continue

            # Now iterate over destinations looking up in the already calculated dictionary
            for dest in all_keys:
                if origin == dest: continue
                
                v = get_osm(dest)
                
                # We check if we reached the destination (as it is a SCC, it should be reachable)
                if v in lengths:
                    length = lengths[v]
                    val = int(round(length))
                    
                    # Fill time matrix
                    matrix_t.append({"from": origin, "to": dest, "value": val})
                    
                    # Fill cost matrix
                    for k in range(1, len(user_vehicles) + 1):
                        matrix_c.append({"k": k, "from": origin, "to": dest, "value": val})
                else:
                    # Should not happen in SCC, but just in case
                    print(f"⚠️ Warning: No path from {u} to {v}")
                    pass

        # --- ASSEMBLE FINAL ---
        data = {
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

    def save_to_file(self, data):
        """Open a file dialog to save the JSON data to a file. Returns the filename or None if cancelled."""
        root = tk.Tk()
        root.withdraw()  # Hide main window

        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save JSON file"
        )
        if filename:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"💾 Saved to {filename}")
            return filename
        else:
            print("❌ Save cancelled.")
            return None