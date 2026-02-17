import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from res_viewer import RouteVisualizer  # Import base class

class CordeauRouteVisualizer(RouteVisualizer):
    """
    Visualizer specifically designed for Cordeau (Euclidean/Cartesian) instances.
    It renders the solution on a 2D plane with specific markers for Pickup/Delivery pairings.
    """

    def plot(self):
        """
        Main method to generate the Cartesian 2D map.
        """
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(10, 8))
        
        print(f"📐 Detected Cordeau instance. Using 2D Plane mode...")

        # 1. Draw static elements (Depot, Requests, P-D relations)
        self._draw_context(ax)

        # 2. Draw dynamic elements (Vehicle Routes)
        self._draw_dynamic_routes(ax)

        # 3. Final configuration (Legend, Grid, Titles)
        self._configure_plot(ax)

        print("✨ Map generated. Opening window...")
        plt.show()

    def _draw_context(self, ax):
        """
        Draws the problem context: Depot, Pickup nodes, Delivery nodes, and their relationships.
        """
        # A. Draw Depot
        if '0' in self.coordinates:
            y_depot, x_depot = self.coordinates['0']
            ax.scatter(x_depot, y_depot, c='black', marker='s', s=150, zorder=5)
            ax.text(x_depot, y_depot + 0.5, 'Depot', fontsize=9, ha='center')

        # B. Draw Requests (Pickup -> Delivery relationships)
        # We iterate from 1 to N_requests to match Cordeau standard IDs
        for i in range(1, self.N_requests + 1):
            p_id = str(i)
            d_id = str(i + self.N_requests)
            
            if p_id in self.coordinates and d_id in self.coordinates:
                # Unpack coordinates (assuming JSON format is [y, x] or [lat, lon])
                py, px = self.coordinates[p_id]
                dy, dx = self.coordinates[d_id]
                
                # Pickup Node (Green Up Triangle)
                ax.scatter(px, py, c='green', marker='^', s=100, edgecolors='black', zorder=4)
                ax.text(px, py + 0.3, f'{i}', fontsize=8, ha='center', color='green', fontweight='bold')
                
                # Delivery Node (Red Down Triangle)
                ax.scatter(dx, dy, c='red', marker='v', s=100, edgecolors='black', zorder=4)
                ax.text(dx, dy + 0.3, f'{i+self.N_requests}', fontsize=8, ha='center', color='red')
                
                # Relationship Line (Dotted Gray)
                ax.plot([px, dx], [py, dy], c='gray', linestyle=':', alpha=0.3, zorder=1)

    def _draw_dynamic_routes(self, ax):
        """
        Draws the actual routes taken by the vehicles with direction arrows.
        """
        print("🎨 Drawing routes on Cartesian plane...")
        
        for i, route in enumerate(self.routes):
            steps = route.get('steps', [])
            color = self.route_colors[i % len(self.route_colors)]
            
            # Extract Path Coordinates
            path_x = []
            path_y = []
            
            for step in steps:
                node_id = str(step['node'])
                if node_id in self.coordinates:
                    y, x = self.coordinates[node_id]
                    path_x.append(x)
                    path_y.append(y)
            
            # A. Draw the continuous route line
            ax.plot(path_x, path_y, color=color, linewidth=2, alpha=0.7, zorder=2)
            
            # B. Draw Direction Arrows
            for k in range(len(path_x) - 1):
                # Calculate vector components
                dx = path_x[k+1] - path_x[k]
                dy = path_y[k+1] - path_y[k]
                
                # Only draw arrow if distance is significant to avoid clutter
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    ax.arrow(
                        path_x[k], path_y[k], 
                        dx * 0.5, dy * 0.5, 
                        head_width=0.3, head_length=0.4, 
                        fc=color, ec=color, alpha=0.8, zorder=3
                    )

    def _configure_plot(self, ax):
        """
        Sets up the legend, titles, and grid.
        """
        ax.set_title(f"Solución DARP - Instancia {self.city}")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, linestyle='--', alpha=0.5)

        # 1. Vehicle Legend
        vehicle_patches = []
        for i, route in enumerate(self.routes):
            vehicle_id = route.get('vehicle_id', i+1)
            color = self.route_colors[i % len(self.route_colors)]
            vehicle_patches.append(mpatches.Patch(color=color, label=f'Vehículo {vehicle_id}'))

        # 2. Node Type Legend
        custom_lines = [
            Line2D([0], [0], marker='s', color='w', markerfacecolor='black', label='Depósito'),
            Line2D([0], [0], marker='^', color='w', markerfacecolor='green', label='Pickup'),
            Line2D([0], [0], marker='v', color='w', markerfacecolor='red', label='Delivery'),
            Line2D([0], [0], color='gray', linestyle=':', label='Flujo Demanda'),
        ]

        ax.legend(handles=custom_lines + vehicle_patches, loc='best')
        plt.tight_layout()
