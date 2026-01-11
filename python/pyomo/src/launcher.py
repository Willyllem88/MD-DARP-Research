import json
import tkinter as tk
from tkinter import filedialog
import sys
import matplotlib.pyplot as plt
import osmnx as ox
import matplotlib.patches as mpatches

from darp_multidepot import DARPMultiDepot

def select_file():
    """Opens a file dialog to select the generated JSON file."""
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Select generated JSON (generated_instance.json)",
        filetypes=[("JSON Files", "*.json")]
    )
    return file_path

def json_to_model_data(json_data):
    """Converts JSON to dictionaries for Python/Pyomo (Original logic intact)"""
    print("⚙️  Processing data...")
    req = json_data['requests']
    
    # Basics
    data = {
        'P': req['pickup_ids'],
        'D': req['delivery_ids'],
        'N': req['n_requests'],
        'K': [], 'Start': {}, 'End': {}, 'Q': {}, 'T_max': {},
        'd': {}, 'q': {}, 'e': {}, 'l': {},
        't': {}, 'c': {},
        'L': json_data['global_params']['L_ride']
    }
    
    # Vehicles
    for v in json_data['vehicles']:
        k = v['id']
        data['K'].append(k)
        data['Start'][k] = v['start_node']
        data['End'][k] = v['end_node']
        data['Q'][k] = v['capacity']
        data['T_max'][k] = v['max_time']
        
    # Nodes
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
        
    return data

def get_node_styles(json_data):
    """
    Analize the JSON to determine the role of each node 
    (Pickup, Delivery, Start, End) and assign the visual style from the Generator.
    """
    styles = {}
    
    # 1. Requests (Pickup = Blue Filled, Delivery = Blue Empty)
    requests = json_data.get('requests_data', []) # Try to find raw data if it exists
    
    # If raw requests_data does not exist (because the JSON only saved processed IDs),
    # we reconstruct the logic based on the IDs.
    # We assume that the request IDs in the output JSON match the indices.
    
    pickup_ids = json_data['requests']['pickup_ids']
    delivery_ids = json_data['requests']['delivery_ids']
    
    # Map Nodes to Roles
    # Requests
    for i, req in enumerate(zip(pickup_ids, delivery_ids)):
        p_id, d_id = req
        # Pickup
        styles[str(p_id)] = {
            'color': 'blue', 'filled': True, 'label': f"{i+1}", 'type': 'Pickup'
        }
        # Delivery
        styles[str(d_id)] = {
            'color': 'blue', 'filled': False, 'label': f"{i+1}", 'type': 'Delivery'
        }

    # Vehicles (Start = Green, End = Red)
    for i, veh in enumerate(json_data['vehicles']):
        s_node = str(veh['start_node'])
        e_node = str(veh['end_node'])
        
        styles[s_node] = {
            'color': 'green', 'filled': True, 'label': f"{i+1}", 'type': 'Start'
        }
        styles[e_node] = {
            'color': 'red', 'filled': True, 'label': f"{i+1}", 'type': 'End'
        }
        
    return styles

def plot_routes_on_map(json_data, route_summary):
    """
    Paints the routes with the same visual style as the generating application.
    """
    city_name = json_data['metadata'].get('city', None)
    coords_map = {str(k): v for k, v in json_data['metadata']['coordinates'].items()}
    
    if not city_name:
        print("⚠️ City name not found. Cannot paint the background.")
        return

    print(f"🗺️  Loading base map: {city_name} ...")
    try:
        # Download graph for visualization
        G = ox.graph_from_place(city_name, network_type='drive')
        fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, 
                                edge_linewidth=0.5, edge_color='#999999', bgcolor='white')
    except Exception as e:
        print(f"Error loading map: {e}")
        return

    # Get node styles (colors and labels identical to the generator)
    node_styles = get_node_styles(json_data)

    # Colors for VEHICLE ROUTES (Different to differentiate them)
    route_colors = ['#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6']
    
    print("🎨 Painting routes and nodes...")
    
    # --- 1. PAINT ROUTES (ARROWS) ---
    legend_patches = []
    
    for idx, k in enumerate(route_summary):
        legs = route_summary[k]
        if not legs: continue
        
        veh_color = route_colors[idx % len(route_colors)]
        legend_patches.append(mpatches.Patch(color=veh_color, label=f'Vehicle {k}'))
        
        for leg in legs:
            u_id = str(leg['from_node'])
            v_id = str(leg['to_node'])
            
            if u_id in coords_map and v_id in coords_map:
                u_y, u_x = coords_map[u_id]
                v_y, v_x = coords_map[v_id]
                
                # Draw route arrow
                ax.annotate("", xy=(v_x, v_y), xytext=(u_x, u_y),
                            arrowprops=dict(arrowstyle="->", color=veh_color, 
                                            lw=2.5, connectionstyle="arc3,rad=0.05"),
                            zorder=2) # Zorder 2: Above map, below nodes

    # --- 2. PAINT NODES (STYLE OF THE GENERATOR) ---
    # Iterate over the nodes defined in the styles
    processed_nodes = set()
    
    for node_id, style in node_styles.items():
        if node_id in coords_map:
            y, x = coords_map[node_id]
            
            color = style['color']
            filled = style['filled']
            label = style['label']
            
            # Marker (Same as in DarpApp: Large circle, colored border)
            ax.plot(
                x, y,
                marker='o',
                markersize=12,
                markeredgecolor=color,
                markerfacecolor=color if filled else 'white', # 'none' sometimes causes issues with zorder, white covers the line
                markeredgewidth=2,
                zorder=3 
            )
            
            # Label (Text with rounded white background)
            ax.text(
                x, y, 
                label, 
                color='black', 
                fontsize=9, 
                fontweight='bold', 
                ha='center', va='center', 
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.7, boxstyle='round,pad=0.2'),
                zorder=4
            )
            processed_nodes.add(node_id)

    # Final configuration
    plt.legend(handles=legend_patches, loc='upper right')
    plt.title(f"DARP Solution - {city_name}", fontsize=14)
    plt.tight_layout()
    
    print("✨ Map generated. Opening window...")
    plt.show()

if __name__ == "__main__":
    print("--- DARP Launcher & Visualizer ---")
    
    # 1. Select File
    path = select_file()
    
    if path:
        with open(path, 'r') as f:
            raw_json = json.load(f)
            
        # 2. Prepare Data
        model_data = json_to_model_data(raw_json)
        
        # 3. Solver
        SOLVER_NAME = 'cplex' 
        print(f"🚀 Running solver: {SOLVER_NAME} ...")
        
        darp = DARPMultiDepot(model_data)
        darp.solve(solver_name=SOLVER_NAME)
        
        # 4. Text Results
        darp.print_route_summary()
        
        # 5. Map Visualization
        summary = darp.get_route_summary()
        
        if isinstance(summary, dict) and summary:
            # Ask or directly show if preferred
            ask = input("\nVisualize map? (y/n) [y]: ")
            if ask.lower() != 'n':
                plot_routes_on_map(raw_json, summary)
        else:
            print("No feasible solution found to plot.")
            
    else:
        print("Operation cancelled.")