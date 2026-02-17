import json
import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import osmnx as ox

class RouteVisualizer:
    def __init__(self, json_data):
        self.data = json_data
        self.city = self.data.get('metadata', {}).get('city', None)
        self.coordinates = self.data.get('metadata', {}).get('coordinates', {})
        self.routes = self.data.get('routes', [])
        self.K_vehicles = self.data.get('summary', {}).get('num_vehicles', len(self.routes))
        self.N_requests = self.data.get('summary', {}).get('num_requests', 0)
        
        # Colors for the routes of different trucks
        self.route_colors = [
            "#1f77b4",
            "#d62728",
            "#2ca02c",
            "#ff7f0e",
            "#9467bd",
            "#8c564b",
            "#17becf",
            "#bcbd22" 
        ]

    def _get_node_style(self, node_type, vehicle_idx):
        """
        Defines the visual style (color, fill) based on the node type.
        """
        style = {
            'color': 'gray',
            'filled': True,
            'label_color': 'black',
            'zorder': 3
        }

        if node_type == 'DepotStart':
            style['color'] = 'green'
            style['filled'] = True
            style['label'] = 'S'  # Start
        elif node_type == 'DepotEnd':
            style['color'] = 'red'
            style['filled'] = True
            style['label'] = 'E'  # End
        elif node_type == 'Pickup':
            style['color'] = 'blue'
            style['filled'] = True # Filled for Pickup
            style['label'] = 'P'
        elif node_type == 'Delivery':
            style['color'] = 'blue'
            style['filled'] = False # Hollow for Delivery
            style['label'] = 'D'
        
        return style

    def _analyze_nodes(self):
        """
        Iterates through routes to determine the role of each node 
        (Pickup, Delivery, Start, End) based on the 'steps'.
        """
        node_styles = {}

        for v_idx, route in enumerate(self.routes):
            steps = route.get('steps', [])
            for step in steps:
                node_id = str(step['node'])
                n_type = step['type']
                
                # Get base style
                style = self._get_node_style(n_type, v_idx)

                def nodeAlias(n_id, n_type):
                    if n_type == 'Pickup':
                        return f"{n_id}"
                    elif n_type == 'Delivery':
                        return f"{n_id - self.N_requests}"
                    else:
                        return f"{(n_id - 2*self.N_requests - 1) % self.K_vehicles + 1}"
                
                # Save the configuration for later drawing
                # Note: If a node is visited multiple times, it keeps the last role 
                # or you can adjust logic if there is overlap.
                node_styles[node_id] = {
                    'style': style,
                    'label': nodeAlias(int(node_id), n_type)
                }
        return node_styles

    def plot(self):
        """
        Main function to generate the map and draw the routes.
        """
        if not self.city:
            print("⚠️ Error: No city found in metadata.")
            return

        print(f"🗺️  Loading base map of: {self.city}...")
        try:
            G = ox.graph_from_place(self.city, network_type='drive')
            fig, ax = ox.plot_graph(G, show=False, close=False, node_size=0, 
                                    edge_linewidth=0.5, edge_color='#999999', bgcolor='white')
        except Exception as e:
            print(f"❌ Error loading the map: {e}")
            return

        print("🎨 Drawing routes and nodes...")

        # 1. Draw Routes (Arrows)
        legend_patches = []
        for i, route in enumerate(self.routes):
            steps = route.get('steps', [])
            vehicle_id = route.get('vehicle_id', i+1)
            color = self.route_colors[i % len(self.route_colors)]
            
            legend_patches.append(Line2D([0], [0], color=color, lw=2, label=f'Vehicle {vehicle_id}'))
            
            for j in range(len(steps) - 1):
                start_node = str(steps[j]['node'])
                end_node = str(steps[j+1]['node'])

                if start_node in self.coordinates and end_node in self.coordinates:
                    y1, x1 = self.coordinates[start_node]
                    y2, x2 = self.coordinates[end_node]
                    # Arrows with a bit of transparency to avoid saturation
                    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                                arrowprops=dict(arrowstyle="->", color=color, 
                                                lw=2, connectionstyle="arc3,rad=0.1", alpha=0.8),
                                zorder=1)

        # 2. Draw Nodes (DARPAPP STYLE)
        node_styles = self._analyze_nodes()
        
        for node_id, data in node_styles.items():
            if node_id in self.coordinates:
                y, x = self.coordinates[node_id]
                style = data['style']
                
                # A. MARKER (Circle)
                ax.plot(
                    x, y,
                    marker='o',
                    markersize=12,
                    markeredgecolor='none' if style['filled'] else style['color'],
                    markerfacecolor=style['color'] if style['filled'] else 'none',
                    markeredgewidth=2,
                    alpha=0.4,
                    zorder=4
                )

                # B. LABEL (Text with background)
                ax.text(
                    x, y, 
                    data['label'], 
                    color='black', 
                    fontsize=10, 
                    fontweight='bold', 
                    ha='center', va='center', 
                )

        # Final configuration
        # Create legend for node types
        node_legend = [
            Line2D([0], [0], marker='o', color='w', label='Depot Start',
                markerfacecolor='green', markeredgecolor='green', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Depot End',
                markerfacecolor='red', markeredgecolor='red', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Pickup',
                markerfacecolor='blue', markeredgecolor='blue', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Delivery',
                markerfacecolor='none', markeredgecolor='blue', markersize=8), # markeredgecolor='blue' y facecolor='none' para hueco
        ]
        plt.legend(handles=legend_patches + node_legend, loc='upper right')
        plt.title(f"Optimized Routes - {self.city}", fontsize=14)
        plt.tight_layout()
        
        print("✨ Map generated. Opening window...")
        plt.show()