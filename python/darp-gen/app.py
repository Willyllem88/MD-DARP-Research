import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from matplotlib.lines import Line2D 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.pyplot as plt
import osmnx as ox

from logic import DARPGraphManager

class MultiFieldDialog(simpledialog.Dialog):
    def __init__(self, parent, title, fields):
        self.fields = fields
        self.vars = {}
        super().__init__(parent, title)

    def body(self, master):
        for i, f in enumerate(self.fields):
            tk.Label(master, text=f['label']).grid(row=i, column=0, sticky="w")
            v = tk.IntVar(value=f.get('default', 0))
            tk.Entry(master, textvariable=v, width=10).grid(row=i, column=1)
            self.vars[f['key']] = v
        return master

    def apply(self):
        self.result = {k: v.get() for k, v in self.vars.items()}


class DarpApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Interactive DARP-MD Instance Generator")
        self.root.geometry("1200x800")
        
        self.manager = DARPGraphManager()
        
        # Variables de estado
        self.step = "CONFIG" # CONFIG, PICKING, DONE
        self.current_pick_target = None # ('req', index, 'pickup'/'delivery') o ('veh', index, 'start'/'end')
        
        self.requests_data = [] # Lista de diccionarios
        self.vehicles_data = [] # Lista de diccionarios
        
        # --- UI LAYOUT ---
        # Panel Izquierdo (Controles)
        self.left_panel = tk.Frame(root, width=300, bg="#f0f0f0")
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        # Panel Derecho (Mapa)
        self.right_panel = tk.Frame(root, bg="white")
        self.right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.setup_config_ui()
        self.setup_map_placeholder()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_config_ui(self):
        # Título
        tk.Label(self.left_panel, text="DARP Configuration", font=("Arial", 14, "bold"), bg="#f0f0f0").pack(pady=10)
        
        # Inputs
        PLACES = [
            "Gracia, Barcelona, Spain",
            "Sarrià-Sant Gervasi, Barcelona, Spain",
            "Chamberí, Madrid, Spain",
            "Salamanca, Madrid, Spain",
            "Barcelona, Spain",
            "Cardedeu, Spain",
            "Westminster, London, UK",
            "Other (write manually)"
        ]
        tk.Label(self.left_panel, text="Place (e.g., Gracia, Barcelona):", bg="#f0f0f0").pack(anchor="w", padx=5)
        self.entry_place = ttk.Combobox(self.left_panel, values=PLACES, state="normal")
        self.entry_place.set(PLACES[0])
        self.entry_place.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Number of Requests:", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_reqs = tk.Entry(self.left_panel)
        self.entry_reqs.insert(0, "2")
        self.entry_reqs.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Number of Vehicles:", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_vehs = tk.Entry(self.left_panel)
        self.entry_vehs.insert(0, "1")
        self.entry_vehs.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Ride Time Max (L_ride):", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_lride = tk.Entry(self.left_panel)
        self.entry_lride.insert(0, "1800")
        self.entry_lride.pack(fill=tk.X, padx=5)

        self.btn_load = tk.Button(self.left_panel, text="Load Map", command=self.load_map, bg="#4CAF50", fg="white")
        self.btn_load.pack(pady=20, fill=tk.X, padx=5)
        
        # Instructions / Status Area
        self.lbl_status = tk.Label(self.left_panel, text="Status: Waiting for config...", fg="blue", bg="#f0f0f0", wraplength=280, font=("Arial", 16))
        self.lbl_status.pack(pady=20, padx=5)

    def setup_map_placeholder(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title("The map will appear here")
        self.ax.axis('off')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_panel)
        self.canvas.draw()

        toolbar = NavigationToolbar2Tk(self.canvas, self.right_panel)
        toolbar.update()

        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Conectar evento click
        self.canvas.mpl_connect('button_press_event', self.on_map_click)

    def load_map(self):
        place = self.entry_place.get()
        try:
            # Lock UI
            self.lbl_status.config(text="Downloading graph... Please wait (this may take a while).", fg="blue", bg="#f0f0f0", wraplength=280)
            self.root.update()
            
            # Download logic
            self.manager.download_graph(place)
            
            # Draw base map
            self.ax.clear()
            # Use osmnx to plot on our 'ax' axis
            ox.plot_graph(self.manager.G, ax=self.ax, show=False, close=False, node_size=0, edge_linewidth=0.5, bgcolor='white')
            
            # We add legend info
            legend_elements = [
                Line2D([0], [0], marker='o', color='w', label='Depot Start',
                       markerfacecolor='green', markeredgecolor='green', markersize=8),
                Line2D([0], [0], marker='o', color='w', label='Depot End',
                       markerfacecolor='red', markeredgecolor='red', markersize=8),
                Line2D([0], [0], marker='o', color='w', label='Pickup',
                       markerfacecolor='blue', markeredgecolor='blue', markersize=8),
                Line2D([0], [0], marker='o', color='w', label='Delivery',
                       markerfacecolor='none', markeredgecolor='blue', markersize=8),
                Line2D([0], [0], color='gray', lw=2, label='Route Path'),
            ]
            self.ax.legend(handles=legend_elements, loc='upper right', fancybox=True, shadow=True)
            
            self.ax.set_title(f"Map: {place}")
            self.canvas.draw()
            
            # Prepare data
            self.n_req = int(self.entry_reqs.get())
            self.n_veh = int(self.entry_vehs.get())
            
            # Initialize empty lists
            self.requests_data = [{} for _ in range(self.n_req)]
            self.vehicles_data = [{} for _ in range(self.n_veh)]
            
            # Start selection sequence
            self.start_selection_sequence()
            
        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.lbl_status.config(text="Error en descarga.")

    def start_selection_sequence(self):
        self.step = "PICKING"
        # Queue of steps: (type, index, subtype)
        self.pick_queue = []
        
        # 1. Requests (Pickup and Delivery)
        for i in range(self.n_req):
            self.pick_queue.append(('req', i, 'pickup'))
            self.pick_queue.append(('req', i, 'delivery'))
            
        # 2. Vehicles (Start and End)
        for i in range(self.n_veh):
            self.pick_queue.append(('veh', i, 'start'))
            self.pick_queue.append(('veh', i, 'end'))
            
        self.process_next_pick()

    def process_next_pick(self):
        if not self.pick_queue:
            self.finish_generation()
            return
            
        self.current_pick_target = self.pick_queue.pop(0)
        tipo, idx, sub = self.current_pick_target
        
        msg = ""
        if tipo == 'req':
            msg = f"Click on the map for: Request {idx+1} ({sub.upper()})"
        else:
            msg = f"Click on the map for: Vehicle {idx+1} ({sub.upper()})"
            
        self.lbl_status.config(text=msg)

    def on_map_click(self, event):
        if self.step != "PICKING" or event.xdata is None or event.ydata is None:
            return
            
        # 1. Get nearest node
        node = self.manager.get_nearest_node(event.ydata, event.xdata) # lat (y), lon (x)
        
        # 2. Mark on map
        y = self.manager.nodes_data[node]['y']
        x = self.manager.nodes_data[node]['x']
        
        tipo, idx, sub = self.current_pick_target
        color_map = {'pickup': 'blue', 'delivery': 'blue', 'start': 'green', 'end': 'red'}
        color = color_map.get(sub, 'black')
        filled = sub != 'delivery'
        
        self.ax.plot(
            x, y,
            marker='o',
            markersize=12,
            markeredgecolor='none' if filled else color,
            markerfacecolor=color if filled else 'none',
            markeredgewidth=2,
            alpha=0.4,
        )
        self.ax.text(
            x, y, 
            str(idx+1), 
            color='black',
            fontsize=10,
            fontweight='bold',
            ha='center', va='center', 
        )
        self.canvas.draw()
        
        # 3. Ask for additional data
        self.ask_node_params(tipo, idx, sub, node)
        
        # 4. Next step
        self.process_next_pick()

    def ask_node_params(self, tipo, idx, sub, node_id):
        """Launch simple popups to ask for data"""
        if tipo == 'req':
            if sub == 'pickup':
                # Pickup questions
                self.requests_data[idx]['pickup_node'] = node_id

                fields = [
                    {'label': f"Demand (Passengers) Request {idx+1}:", 'key': 'demand', 'default': 1},
                    {'label': f"Time Window Start (Pickup {idx+1}):", 'key': 'p_tw_start', 'default': 0},
                    {'label': f"Time Window End (Pickup {idx+1}):", 'key': 'p_tw_end', 'default': 1440},
                    {'label': f"Service Time (Pickup {idx+1}):", 'key': 'p_service_time', 'default': 60},
                ]
                dialog = MultiFieldDialog(self.root, f"Pickup Data {idx+1}", fields)
                if dialog.result:
                    self.requests_data[idx].update(dialog.result)
                else:
                    # Defaults if dialog cancelled
                    self.requests_data[idx].update({
                        'demand': 1,
                        'p_tw_start': 0,
                        'p_tw_end': 1440,
                        'p_service_time': 60
                    })
                
            else: # delivery
                self.requests_data[idx]['delivery_node'] = node_id
                
                self.requests_data[idx]['delivery_node'] = node_id

                fields = [
                    {'label': f"Time Window Start (Delivery {idx+1}):", 'key': 'd_tw_start', 'default': 0},
                    {'label': f"Time Window End (Delivery {idx+1}):", 'key': 'd_tw_end', 'default': 1440},
                    {'label': f"Service Time (Delivery {idx+1}):", 'key': 'd_service_time', 'default': 60},
                ]
                dialog = MultiFieldDialog(self.root, f"Delivery Data {idx+1}", fields)
                if dialog.result:
                    self.requests_data[idx].update(dialog.result)
                else:
                    self.requests_data[idx].update({
                        'd_tw_start': 0,
                        'd_tw_end': 1440,
                        'd_service_time': 60
                    })

        elif tipo == 'veh':
            if sub == 'start':
                self.vehicles_data[idx]['start_node'] = node_id

                fields = [
                    {'label': f"Vehicle Capacity {idx+1}:", 'key': 'capacity', 'default': 3},
                    {'label': f"Vehicle Max Route Time {idx+1}:", 'key': 'max_time', 'default': 6000},
                ]
                dialog = MultiFieldDialog(self.root, f"Vehicle Data {idx+1}", fields)
                if dialog.result:
                    self.vehicles_data[idx].update(dialog.result)
                else:
                    self.vehicles_data[idx].update({
                        'capacity': 3,
                        'max_time': 6000
                    })
            else:
                self.vehicles_data[idx]['end_node'] = node_id

    def finish_generation(self):
        self.step = "DONE"
        self.lbl_status.config(text="Processing graphs and generating JSON...")
        self.root.update()
        
        # Global parameters
        l_ride = int(self.entry_lride.get())
        global_params = {"L_ride": l_ride}
        
        # Call backend
        try:
            json_data = self.manager.generate_json_structure(
                self.requests_data, 
                self.vehicles_data, 
                global_params
            )
            
            filename = self.manager.save_to_file(json_data)
            if filename:
                messagebox.showinfo("Success", f"File generated successfully:\n{filename}")
                self.lbl_status.config(text="JSON Generated. Close or restart for another.")
            else:
                messagebox.showinfo("Cancelled", "File save cancelled.")
                self.lbl_status.config(text="JSON Cancelled. Close or restart for another.")
                 
        except Exception as e:
            messagebox.showerror("Error Generating JSON", str(e))
            self.lbl_status.config(text="Fatal error.")

    def on_closing(self):
        """Handle proper closing of the application."""
        try:
            plt.close('all') 
            self.root.quit()             
            self.root.destroy() 
        except Exception as e:
            print(f"Error when loading: {e}")
            import sys; sys.exit(0)
            
if __name__ == "__main__":
    root = tk.Tk()
    app = DarpApp(root)
    root.mainloop()