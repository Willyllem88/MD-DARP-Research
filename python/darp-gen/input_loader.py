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

        # 1. Configurar Grafo (Lugar)
        place = data.get('place', 'Barcelona, Spain')
        
        # Si el grafo no está cargado o es un sitio diferente, descargarlo
        # (Asumimos que manager tiene un atributo para saber el sitio actual o simplemente recargamos)
        try:
            print(f"Cargando mapa para: {place}...")
            self.manager.download_graph(place)
        except Exception as e:
            messagebox.showerror("Error Mapa", f"No se pudo descargar el mapa de {place}:\n{e}")
            return None

        requests_data = []
        vehicles_data = []

        # 2. Process Requests
        print("Geocoding Requests...")
        for req in data.get('requests', []):
            try:
                # Geocode Pickup
                p_addr = req.get('pickup_address')
                p_node = self._address_to_node(p_addr)
                
                # Geocode Delivery
                d_addr = req.get('delivery_address')
                d_node = self._address_to_node(d_addr)

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
                    # Extra metadata for visualization if needed
                    'meta_pickup_addr': p_addr,
                    'meta_delivery_addr': d_addr
                }
                requests_data.append(r_obj)
            except Exception as e:
                print(f"Error processing Request {req.get('id')}: {e}")
                continue

        # 3. Procesar Vehicles
        print("Geocoding Vehicles...")
        for veh in data.get('vehicles', []):
            try:
                s_addr = veh.get('start_address')
                s_node = self._address_to_node(s_addr)
                
                e_addr = veh.get('end_address')
                e_node = self._address_to_node(e_addr)

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
            # If you don't do this, your IP will be blocked while processing the list.
            time.sleep(2.0) 
            
        except Exception as e:
            raise ValueError(f"Error geocoding '{address}': {e}")

        # El resto sigue igual (buscar nodo en el grafo del manager)
        node_dist = self.manager.get_nearest_node_distance(lat, lon)
        if node_dist > 500:
            print(f"⚠️ Warning: the address '{address}' is {node_dist:.1f} meters from the nearest graph node. Consider refining the address for better accuracy.")
        node = self.manager.get_nearest_node(lat, lon)
        return node