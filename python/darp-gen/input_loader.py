import yaml
from geopy.geocoders import Nominatim
import time
from tkinter import messagebox

class YamlLoader:
    def __init__(self, manager):
        self.manager = manager
        self.geolocator = Nominatim(user_agent="darp_gen_app")

    def load_scenario(self, filepath):
        """
        Read the YAML, download the map (if changed), geocode addresses
        and return the data structures ready for the app.
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            messagebox.showerror("Error YAML", f"Could not read the file:\n{e}")
            return None

        place = data.get('place', 'Barcelona, Spain')
        
        try:
            print(f"Loading map for: {place}...")
            self.manager.download_graph(place)
        except Exception as e:
            messagebox.showerror("Error Map", f"Could not download the map for {place}:\n{e}")
            return None

        requests_data = []
        vehicles_data = []

        # 2. Process Requests
        print("Geocoding Requests...")
        for req in data.get('requests', []):
            try:
                # Geocode Pickup
                p_coords = req.get('pickup_coords')
                p_node = self._coords_to_node(p_coords) if p_coords else self._address_to_node(req.get('pickup_address'))

                # Geocode Delivery
                d_coords = req.get('delivery_coords')
                d_node = self._coords_to_node(d_coords) if d_coords else self._address_to_node(req.get('delivery_address'))

                # Build request object
                r_obj = {
                    'pickup_node': p_node,
                    'delivery_node': d_node,
                    'demand': req.get('demand', 1),
                    'p_tw_start': req.get('p_tw_start', 0),
                    'p_tw_end': req.get('p_tw_end', 1440),
                    'p_service_time': req.get('p_service_time', 60),
                    'd_tw_start': req.get('d_tw_start', 0),
                    'd_tw_end': req.get('d_tw_end', 1440),
                    'd_service_time': req.get('d_service_time', 60),
                }
                requests_data.append(r_obj)
            except Exception as e:
                print(f"Error processing Request {req.get('id')}: {e}")
                continue

        # 3. Process Vehicles
        print("Geocoding Vehicles...")
        for veh in data.get('vehicles', []):
            try:
                s_addr = veh.get('start_coords')
                s_node = self._coords_to_node(s_addr) if s_addr else self._address_to_node(veh.get('start_address'))

                e_addr = veh.get('end_coords')
                e_node = self._coords_to_node(e_addr) if e_addr else self._address_to_node(veh.get('end_address'))

                v_obj = {
                    'start_node': s_node,
                    'end_node': e_node,
                    'capacity': veh.get('capacity', 3),
                    'vstart_tw_start': veh.get('vstart_tw_start', 0),
                    'vstart_tw_end': veh.get('vstart_tw_end', 10000),
                    'vend_tw_start': veh.get('vend_tw_start', 0),
                    'vend_tw_end': veh.get('vend_tw_end', 10000),
                    'max_time': veh.get('max_time', 6000),
                    'meta_start_addr': s_addr,
                    'meta_end_addr': e_addr
                }
                vehicles_data.append(v_obj)
            except Exception as e:
                print(f"Error processing Vehicle {veh.get('id')}: {e}")
                continue
        
        global_params = data.get('global', {'L_ride': 1800})
        
        return {
            'requests': requests_data,
            'vehicles': vehicles_data,
            'global': global_params,
            'place': place
        }

    def _coords_to_node(self, coords):
        """Convert lat/lon to Node ID using the manager's graph"""
        lat, lon = coords
        node_dist = self.manager.get_nearest_node_distance(lat, lon)
        if node_dist > 500:
            print(f"⚠️ Warning: the coordinates ({lat}, {lon}) are {node_dist:.1f} meters from the nearest graph node. Consider refining the coordinates for better accuracy.")
        node = self.manager.get_nearest_node(lat, lon)
        return node

    def _address_to_node(self, address):
        """Convert text address to Node ID using Geopy"""
        if not address:
            raise ValueError("Empty address")
        
        try:
            # Call to Geopy
            location = self.geolocator.geocode(address)
            
            if location is None:
                raise ValueError(f"Geopy did not find coordinates for: {address}")
            
            lat = location.latitude
            lon = location.longitude
            
            # IMPORTANT: Pause to respect Nominatim's policy (1 req/sec)
            time.sleep(1.1)  # Sleep a bit more than 1 second to be safe
            
        except Exception as e:
            raise ValueError(f"Error geocoding '{address}': {e}")

        node_dist = self.manager.get_nearest_node_distance(lat, lon)
        if node_dist > 500:
            print(f"⚠️ Warning: the address '{address}' is {node_dist:.1f} meters from the nearest graph node. Consider refining the address for better accuracy.")
        node = self.manager.get_nearest_node(lat, lon)
        return node