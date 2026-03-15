import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Any

@dataclass
class Metadata:
    city: str = ""
    coordinates: Dict[str, List[float]] = field(default_factory=dict)

    def is_empty(self) -> bool:
        """
        Devuelve True si tanto el nombre de la ciudad como las coordenadas están vacíos.
        """
        return not self.city and not self.coordinates

@dataclass
class MDDARP_ProblemInstance:
    N_requests: int = 0
    K_vehicles: int = 0
    max_node_id: int = 0
    max_ride_time: float = 0.0
    INF: float = 1e9

    # Sets
    P: List[int] = field(default_factory=list)
    D: List[int] = field(default_factory=list)
    K: List[int] = field(default_factory=list)
    StartNodes: List[int] = field(default_factory=list)
    EndNodes: List[int] = field(default_factory=list)

    # Vehicle mapping (k -> node_id)
    start_node: Dict[int, int] = field(default_factory=dict)
    end_node: Dict[int, int] = field(default_factory=dict)

    # Node attributes (node_id -> value)
    service_time: Dict[int, float] = field(default_factory=dict)
    demand: Dict[int, float] = field(default_factory=dict)
    time_window_start: Dict[int, float] = field(default_factory=dict)
    time_window_end: Dict[int, float] = field(default_factory=dict)

    # Vehicle attributes (veh_id -> value)
    capacity: Dict[int, float] = field(default_factory=dict)
    max_route_time: Dict[int, float] = field(default_factory=dict)

    # Matrixes using tuples as keys: (i, j) for t_ij and (i, j, k) for c_ijk
    t_ij: Dict[Tuple[int, int], float] = field(default_factory=dict)
    c_ijk: Dict[Tuple[int, int, int], float] = field(default_factory=dict)

    metadata: Metadata = field(default_factory=Metadata)

    def clear(self):
        """Resets the instance to its default values."""
        self.N_requests = 0
        self.K_vehicles = 0
        self.max_node_id = 0
        self.max_ride_time = 0.0
        self.P.clear()
        self.D.clear()
        self.K.clear()
        self.StartNodes.clear()
        self.EndNodes.clear()
        self.start_node.clear()
        self.end_node.clear()
        self.service_time.clear()
        self.demand.clear()
        self.time_window_start.clear()
        self.time_window_end.clear()
        self.capacity.clear()
        self.max_route_time.clear()
        self.t_ij.clear()
        self.c_ijk.clear()
        self.metadata = Metadata()

    def load_from_json(self, path: str) -> bool:
        """Loads data from a JSON file."""
        self.clear()
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                j = json.load(f)
            
            print("Cargando instancia...")

            # 1. Requests and basic parameters
            req = j["requests"]
            self.P = req["pickup_ids"]
            self.D = req["delivery_ids"]
            self.N_requests = req["n_requests"]
            self.K_vehicles = len(j["vehicles"])
            self.max_node_id = 2 * self.N_requests + 2 * self.K_vehicles

            # 2. Nodes
            for n in j["nodes"]:
                nid = n["id"]
                self.service_time[nid] = n["service_time"]
                self.demand[nid] = n["demand"]
                self.time_window_start[nid] = n["tw_start"]
                self.time_window_end[nid] = n["tw_end"]

            # 3. Vehicles
            for v in j["vehicles"]:
                k = v["id"]
                self.K.append(k)
                self.StartNodes.append(v["start_node"])
                self.EndNodes.append(v["end_node"])
                self.start_node[k] = v["start_node"]
                self.end_node[k] = v["end_node"]
                self.capacity[k] = v["capacity"]
                self.max_route_time[k] = v["max_time"]

            # 4. Global Parameters
            self.max_ride_time = j["global_params"]["L_ride"]

            # 5. Travel Time Matrix (t_ij) -> Dictionary (i, j)
            for x in j["matrix_t"]:
                self.t_ij[(x["from"], x["to"])] = x["value"]

            # 6. Cost Matrix (c_ijk) -> Dictionary (i, j, k)
            for x in j["matrix_c"]:
                self.c_ijk[(x["from"], x["to"], x["k"])] = x["value"]

            # 7. Metadata
            if "metadata" in j:
                self.metadata = Metadata(**j["metadata"])

            return True

        except FileNotFoundError:
            print(f"Error: File {path} not found.")
            return False
        except Exception as e:
            print(f"[ERROR] An error occurred while parsing the JSON: {e}")
            return False

    def display_info(self):
        """Show the information of the instance by console."""
        print("\n===========================================")
        print("       MD-DARP Problem Instance Info         ")
        print("=============================================")

        print("--- Global Parameters ---")
        print(f"{'Number of Requests:':<25}{self.N_requests}")
        print(f"{'Max Node ID:':<25}{self.max_node_id}")
        print(f"{'Max Ride Time (L):':<25}{self.max_ride_time}\n")

        print("--- Sets ---")
        print(f"Pickups (P): {self.P} (Size: {len(self.P)})")
        print(f"Deliveries (D): {self.D} (Size: {len(self.D)})")
        print(f"Vehicles (K): {self.K} (Size: {len(self.K)})\n")

        print("--- Vehicle Configuration (K) ---")
        print(f"{'Veh_ID':<10}{'Start_Node':<12}{'End_Node':<12}{'Capacity':<12}Max_Route_Time")
        print("-" * 60)
        for k in self.K:
            start = self.start_node.get(k, -1)
            end = self.end_node.get(k, -1)
            cap = self.capacity.get(k, 0.0)
            max_t = self.max_route_time.get(k, 0.0)
            print(f"{k:<10}{start:<12}{end:<12}{cap:<12}{max_t}")
        print()

        print("--- Node Attributes ---")
        print(f"{'Node_ID':<10}{'Service_T':<15}{'Demand':<10}{'TW_Start':<15}TW_End")
        print("-" * 60)
        
        all_nodes = set(list(self.start_node.values()) + list(self.end_node.values()) + self.P + self.D)
        for n in sorted(all_nodes):
            st = self.service_time.get(n, 0.0)
            d = self.demand.get(n, 0.0)
            tws = self.time_window_start.get(n, 0.0)
            twe = self.time_window_end.get(n, 0.0)
            print(f"{n:<10}{st:<15}{d:<10}{tws:<15}{twe}")
        print()

        print("--- Matrices Dimensions ---")
        print(f"Travel Time Matrix elements stored: {len(self.t_ij)}")
        print(f"Cost Matrix elements stored: {len(self.c_ijk)}")
        print("============================================\n")

    # Getters
    def get_service_time(self, i: int) -> float: return self.service_time.get(i, 0.0)
    def get_demand(self, i: int) -> float: return self.demand.get(i, 0.0)
    def get_time_window_start(self, i: int) -> float: return self.time_window_start.get(i, 0.0)
    def get_time_window_end(self, i: int) -> float: return self.time_window_end.get(i, 0.0)
    
    def get_vehicle_capacity(self, k: int) -> float: return self.capacity.get(k, 0.0)
    def get_vehicle_max_route_time(self, k: int) -> float: return self.max_route_time.get(k, 0.0)

    # For matrices we use .get() returning INF if the route does not exist
    def get_travel_time(self, i: int, j: int) -> float: return self.t_ij.get((i, j), self.INF)
    def get_cost(self, i: int, j: int, k: int) -> float: return self.c_ijk.get((i, j, k), self.INF)
        

    # Helpers for node logic
    def is_pickup(self, i: int) -> bool: return 1 <= i <= self.N_requests 
    def is_delivery(self, i: int) -> bool: return self.N_requests < i <= 2 * self.N_requests  
    def is_vehicle_start(self, i: int) -> bool: return 2 * self.N_requests < i <= 2 * self.N_requests + self.K_vehicles 
    def is_vehicle_end(self, i: int) -> bool: return 2 * self.N_requests + self.K_vehicles < i <= self.max_node_id

    # Setters
    def update_time_window(self, i: int, new_start: float, new_end: float):
        """Update the time windows for a given node."""
        if i in self.time_window_start and i in self.time_window_end:
            self.time_window_start[i] = new_start
            self.time_window_end[i] = new_end
        else:
            print(f"Warning: Node {i} does not exist in time windows.")