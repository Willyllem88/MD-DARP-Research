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
        # A. Draw First Depot (as all vehicles start from the same depot in Cordeau instances)
        first_depot_index = str(2 * self.N_requests + 1)
        print(f"🏠 Drawing depot at node {first_depot_index}...")
        y_depot, x_depot = self.coordinates[first_depot_index]
        ax.scatter(x_depot, y_depot, c='black', marker='s', s=150, zorder=5)

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
                ax.plot(
                    px, py,
                    marker='o', markersize=12,
                    markeredgecolor='none', markerfacecolor='blue',
                    markeredgewidth=2, alpha=0.4, zorder=4
                )
                ax.text(
                    px, py, f'{i}', 
                    color='black', fontsize=10, fontweight='bold', 
                    ha='center', va='center', zorder=4
                )

                # Delivery Node (Red Down Triangle)
                ax.plot(
                    dx, dy,
                    marker='o', markersize=12,
                    markeredgecolor='blue', markerfacecolor='none',
                    markeredgewidth=2, alpha=0.4, zorder=1
                )
                ax.text(
                    dx, dy, f'{i}',
                    color='black', fontsize=10, fontweight='bold', 
                    ha='center', va='center', zorder=1
                )

                # Relationship Line (Dotted Gray)
                ax.plot([px, dx], [py, dy], c='gray', linestyle=':', alpha=0.3, zorder=1)

    def _draw_dynamic_routes(self, ax):
        """
        Draws the actual routes taken by the vehicles with direction arrows.
        """
        print("🎨 Drawing routes on Cartesian plane...")
        
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
                        

    def _configure_plot(self, ax):
        """
        Sets up the legend, titles, and grid.
        """
        ax.set_title(f"MD-DARP Solution - (Cordeau Instance)")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, linestyle='--', alpha=0.5)

        # 1. Vehicle Legend
        vehicle_patches = []
        for i, route in enumerate(self.routes):
            vehicle_id = route.get('vehicle_id', i+1)
            color = self.route_colors[i % len(self.route_colors)]
            vehicle_patches.append(mpatches.Patch(color=color, label=f'Vehicle {vehicle_id}'))

        # 2. Node Type Legend
        custom_lines = [
            Line2D([0], [0], marker='o', color='w', label='Pickup',
                markerfacecolor='blue', markeredgecolor='blue', markersize=8),
            Line2D([0], [0], marker='o', color='w', label='Delivery',
                markerfacecolor='none', markeredgecolor='blue', markersize=8),
            Line2D([0], [0], marker='s', color='w', label='Depot',
                   markerfacecolor='black', markeredgecolor='black', markersize=8),
            Line2D([0], [0], linestyle=':', color='gray', label='P-D Relationship')
        ]

        ax.legend(handles=custom_lines + vehicle_patches, loc='best')
        plt.tight_layout()
