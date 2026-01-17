
import json
import tkinter as tk
from tkinter import filedialog
from res_viewer import RouteVisualizer

def load_json_file():
    """
    Opens a tkinter dialog to select a file.
    """
    root = tk.Tk()
    root.withdraw() # Hide the main tkinter window
    
    file_path = filedialog.askopenfilename(
        title="Select the routes JSON file",
        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
    )
    
    if file_path:
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    return None

if __name__ == "__main__":
    # 1. Load JSON via dialog window
    data = load_json_file()
    
    if data:
        # 2. Instantiate visualizer
        visualizer = RouteVisualizer(data)
        
        # 3. Plot
        visualizer.plot()
    else:
        print("Operation cancelled or invalid file.")