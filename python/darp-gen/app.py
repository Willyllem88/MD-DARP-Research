import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import osmnx as ox

# Importamos la lógica del otro archivo
from logic import DARPGraphManager

class DarpApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Generador Interactivo DARP")
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

    def setup_config_ui(self):
        # Título
        tk.Label(self.left_panel, text="Configuración DARP", font=("Arial", 14, "bold"), bg="#f0f0f0").pack(pady=10)
        
        # Inputs
        tk.Label(self.left_panel, text="Lugar (ej: Gracia, Barcelona):", bg="#f0f0f0").pack(anchor="w", padx=5)
        self.entry_place = tk.Entry(self.left_panel)
        self.entry_place.insert(0, "Gracia, Barcelona, Spain")
        self.entry_place.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Nº Peticiones:", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_reqs = tk.Entry(self.left_panel)
        self.entry_reqs.insert(0, "2")
        self.entry_reqs.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Nº Vehículos:", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_vehs = tk.Entry(self.left_panel)
        self.entry_vehs.insert(0, "1")
        self.entry_vehs.pack(fill=tk.X, padx=5)
        
        tk.Label(self.left_panel, text="Ride Time Max (L_ride):", bg="#f0f0f0").pack(anchor="w", padx=5, pady=(10,0))
        self.entry_lride = tk.Entry(self.left_panel)
        self.entry_lride.insert(0, "1800")
        self.entry_lride.pack(fill=tk.X, padx=5)

        self.btn_load = tk.Button(self.left_panel, text="Cargar Mapa", command=self.load_map, bg="#4CAF50", fg="white")
        self.btn_load.pack(pady=20, fill=tk.X, padx=5)
        
        # Area de instrucciones / Estado
        self.lbl_status = tk.Label(self.left_panel, text="Estado: Esperando config...", fg="blue", bg="#f0f0f0", wraplength=280)
        self.lbl_status.pack(pady=20, padx=5)

    def setup_map_placeholder(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title("El mapa aparecerá aquí")
        self.ax.axis('off')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Conectar evento click
        self.canvas.mpl_connect('button_press_event', self.on_map_click)

    def load_map(self):
        place = self.entry_place.get()
        try:
            # Bloquear UI
            self.lbl_status.config(text="Descargando grafo... Por favor espera (puede tardar).")
            self.root.update()
            
            # Descargar lógica
            self.manager.download_graph(place)
            
            # Dibujar mapa base
            self.ax.clear()
            # Usamos osmnx para plotear sobre nuestro eje 'ax'
            ox.plot_graph(self.manager.G, ax=self.ax, show=False, close=False, node_size=0, edge_linewidth=0.5, bgcolor='white')
            self.ax.set_title(f"Mapa: {place}")
            self.canvas.draw()
            
            # Preparar datos
            self.n_req = int(self.entry_reqs.get())
            self.n_veh = int(self.entry_vehs.get())
            
            # Inicializar listas vacías
            self.requests_data = [{} for _ in range(self.n_req)]
            self.vehicles_data = [{} for _ in range(self.n_veh)]
            
            # Iniciar secuencia de selección
            self.start_selection_sequence()
            
        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.lbl_status.config(text="Error en descarga.")

    def start_selection_sequence(self):
        self.step = "PICKING"
        # Cola de pasos: (tipo, indice, subtipo)
        self.pick_queue = []
        
        # 1. Peticiones (Pickup y Delivery)
        for i in range(self.n_req):
            self.pick_queue.append(('req', i, 'pickup'))
            self.pick_queue.append(('req', i, 'delivery'))
            
        # 2. Vehículos (Start y End)
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
            msg = f"👉 Haz CLIC en el mapa para: Petición {idx+1} ({sub.upper()})"
        else:
            msg = f"👉 Haz CLIC en el mapa para: Vehículo {idx+1} ({sub.upper()})"
            
        self.lbl_status.config(text=msg, fg="red", font=("Arial", 12, "bold"))

    def on_map_click(self, event):
        if self.step != "PICKING" or event.xdata is None or event.ydata is None:
            return
            
        # 1. Obtener nodo más cercano
        node = self.manager.get_nearest_node(event.ydata, event.xdata) # lat (y), lon (x)
        
        # 2. Marcar en mapa
        y = self.manager.nodes_data[node]['y']
        x = self.manager.nodes_data[node]['x']
        
        tipo, idx, sub = self.current_pick_target
        color = 'blue' if tipo == 'req' and sub == 'pickup' else 'orange' if tipo == 'req' else 'green'
        marker = 'o' if sub == 'pickup' or sub == 'start' else 'x'
        
        self.ax.plot(x, y, marker=marker, color=color, markersize=10, markeredgecolor='black')
        self.canvas.draw()
        
        # 3. Pedir datos adicionales
        self.ask_node_params(tipo, idx, sub, node)
        
        # 4. Siguiente paso
        self.process_next_pick()

    def ask_node_params(self, tipo, idx, sub, node_id):
        """Lanza popups simples para pedir datos"""
        if tipo == 'req':
            if sub == 'pickup':
                # Preguntas Pickup
                self.requests_data[idx]['pickup_node'] = node_id
                d = simpledialog.askinteger("Datos", f"Demanda (Pasajeros) Petición {idx+1}:", initialvalue=1, minvalue=1)
                self.requests_data[idx]['demand'] = d if d else 1
                
                tw_s = simpledialog.askinteger("Datos", f"Ventana Tiempo Inicio (Pickup {idx+1}):", initialvalue=0)
                self.requests_data[idx]['p_tw_start'] = tw_s if tw_s is not None else 0
                
                tw_e = simpledialog.askinteger("Datos", f"Ventana Tiempo Fin (Pickup {idx+1}):", initialvalue=1440)
                self.requests_data[idx]['p_tw_end'] = tw_e if tw_e else 1440
                
                st = simpledialog.askinteger("Datos", f"Tiempo de servicio (Pickup {idx+1}):", initialvalue=60)
                self.requests_data[idx]['p_service_time'] = st if st is not None else 60
                
            else: # delivery
                self.requests_data[idx]['delivery_node'] = node_id
                
                tw_s = simpledialog.askinteger("Datos", f"Ventana Tiempo Inicio (Delivery {idx+1}):", initialvalue=0)
                self.requests_data[idx]['d_tw_start'] = tw_s if tw_s is not None else 0
                
                tw_e = simpledialog.askinteger("Datos", f"Ventana Tiempo Fin (Delivery {idx+1}):", initialvalue=1440)
                self.requests_data[idx]['d_tw_end'] = tw_e if tw_e else 1440
                
                st = simpledialog.askinteger("Datos", f"Tiempo de servicio (Delivery {idx+1}):", initialvalue=60)
                self.requests_data[idx]['d_service_time'] = st if st is not None else 60

        elif tipo == 'veh':
            if sub == 'start':
                self.vehicles_data[idx]['start_node'] = node_id
                
                cap = simpledialog.askinteger("Datos", f"Capacidad Vehículo {idx+1}:", initialvalue=3)
                self.vehicles_data[idx]['capacity'] = cap if cap else 3
                
                mt = simpledialog.askinteger("Datos", f"Tiempo Máximo Ruta Vehículo {idx+1}:", initialvalue=6000)
                self.vehicles_data[idx]['max_time'] = mt if mt else 6000
            else:
                self.vehicles_data[idx]['end_node'] = node_id

    def finish_generation(self):
        self.step = "DONE"
        self.lbl_status.config(text="Procesando grafos y generando JSON...", fg="blue")
        self.root.update()
        
        # Parámetros globales
        l_ride = int(self.entry_lride.get())
        global_params = {"L_ride": l_ride}
        
        # Llamar al backend
        try:
            json_data = self.manager.generate_json_structure(
                self.requests_data, 
                self.vehicles_data, 
                global_params
            )
            
            filename = "generated_instance.json"
            self.manager.save_to_file(json_data, filename)
            
            messagebox.showinfo("Éxito", f"Archivo generado correctamente:\n{filename}")
            self.lbl_status.config(text="JSON Generado. Cierra o reinicia para otro.")
            
        except Exception as e:
            messagebox.showerror("Error Generando JSON", str(e))
            self.lbl_status.config(text="Error fatal.")

if __name__ == "__main__":
    root = tk.Tk()
    app = DarpApp(root)
    root.mainloop()