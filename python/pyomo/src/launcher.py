import json
import tkinter as tk
from tkinter import filedialog
import sys
import os
import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx

# Importar tu clase modelo
try:
    from darp_multidepot import DARPMultiDepot
except ImportError:
    print("❌ Error: No se encuentra 'darp_multidepot.py'.")
    sys.exit(1)

def select_file():
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Selecciona JSON (barcelona_darp.json)",
        filetypes=[("JSON Files", "*.json")]
    )
    return file_path

def json_to_model_data(json_data):
    """Convierte JSON a diccionarios para Python/Pyomo"""
    print("⚙️  Procesando datos...")
    req = json_data['requests']
    
    # Básicos
    data = {
        'P': req['pickup_ids'],
        'D': req['delivery_ids'],
        'N': req['n_requests'],
        'K': [], 'Start': {}, 'End': {}, 'Q': {}, 'T_max': {},
        'd': {}, 'q': {}, 'e': {}, 'l': {},
        't': {}, 'c': {},
        'L': json_data['global_params']['L_ride']
    }
    
    # Vehículos
    for v in json_data['vehicles']:
        k = v['id']
        data['K'].append(k)
        data['Start'][k] = v['start_node']
        data['End'][k] = v['end_node']
        data['Q'][k] = v['capacity']
        data['T_max'][k] = v['max_time']
        
    # Nodos
    for n in json_data['nodes']:
        nid = n['id']
        data['d'][nid] = n['service_time']
        data['q'][nid] = n['demand']
        data['e'][nid] = n['tw_start']
        data['l'][nid] = n['tw_end']
        
    # Matrices
    for x in json_data['matrix_t']:
        data['t'][(x['from'], x['to'])] = x['value']
    for x in json_data['matrix_c']:
        data['c'][(x['from'], x['to'], x['k'])] = x['value']
        
    return data, json_data.get('solver_options', {})

def plot_routes_on_map(json_data, route_summary):
    """
    Pinta las rutas sobre el mapa real usando OSMnx y Matplotlib.
    """
    city_name = json_data['metadata'].get('city', None)
    coords_map = json_data['metadata']['coordinates']
    
    if not city_name:
        print("⚠️ No se encontró nombre de ciudad en metadata. No se puede pintar el fondo.")
        return

    print(f"🗺️  Preparando mapa de fondo: {city_name} (esto puede tardar)...")
    try:
        # Descargar grafo SOLO para pintar el fondo (visualización)
        G = ox.graph_from_place(city_name, network_type='drive')
        fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, edge_color='#999999', edge_alpha=0.5)
    except Exception as e:
        print(f"Error cargando mapa: {e}")
        return

    # Definir colores para vehículos
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'cyan']
    
    print("🎨 Pintando rutas...")
    
    for idx, k in enumerate(route_summary):
        legs = route_summary[k]
        if not legs: continue
        
        color = colors[idx % len(colors)]
        
        # Recorrer cada tramo de la ruta
        for leg in legs:
            u_id = str(leg['from_node']) # El JSON guarda claves como string a veces
            v_id = str(leg['to_node'])
            
            # Obtener Lat/Lon
            # Nota: coordenadas suelen guardarse como (y, x) o (lat, lon)
            if u_id not in coords_map or v_id not in coords_map:
                continue
                
            u_y, u_x = coords_map[u_id]
            v_y, v_x = coords_map[v_id]
            
            # Pintar la linea (Flecha)
            # Zorder alto para que salga encima del mapa
            ax.arrow(u_x, u_y, v_x - u_x, v_y - u_y, 
                     fc=color, ec=color, width=0.0001, 
                     head_width=0.0005, head_length=0.0005, 
                     length_includes_head=True, zorder=5)
            
            # Pintar nodos (Origen: Punto, Destino: X)
            ax.scatter(u_x, u_y, c='black', s=20, zorder=6) # Node

    plt.title(f"Solución DARP - {city_name}")
    print("✨ Mapa generado. Abriendo ventana...")
    plt.show()

if __name__ == "__main__":
    print("--- DARP Launcher Completo ---")
    
    # 1. Seleccionar Archivo
    path = select_file()
    
    if path:
        with open(path, 'r') as f:
            raw_json = json.load(f)
            
        # 2. Preparar Datos
        model_data, opts = json_to_model_data(raw_json)
        
        # 3. Solver
        solver_name = opts.get('solver_name', 'glpk') # Asegúrate de tener glpk o cplex
        print(f"🚀 Ejecutando solver: {solver_name}...")
        
        darp = DARPMultiDepot(model_data)
        darp.solve(solver_name=solver_name, time_limit=opts.get('time_limit', 60))
        
        # 4. Resultados Texto
        darp.print_route_summary()
        
        # 5. Visualización Mapa
        # Obtenemos el diccionario resumen de la clase
        summary = darp.get_route_summary()
        
        # Si no es string (error), pintamos
        if isinstance(summary, dict):
            ask = input("\n¿Quieres visualizar las rutas en el mapa? (s/n): ")
            if ask.lower() == 's':
                plot_routes_on_map(raw_json, summary)
    else:
        print("Operación cancelada.")