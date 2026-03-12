import json
import tkinter as tk
from tkinter import filedialog
import argparse
import sys
import os

from res_viewer import RouteVisualizer
from cordeau_viewer import CordeauRouteVisualizer

def load_json_from_path(file_path):
    """
    Try to load the JSON from a specific path.
    Returns the data or None if there is an error.
    """
    if not os.path.isfile(file_path):
        print(f"Error: The file '{file_path}' does not exist.")
        return None

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            print(f"Loading file: {file_path}")
            return json.load(f)
    except json.JSONDecodeError:
        print(f"Error: The file '{file_path}' is not a valid JSON.")
        return None
    except Exception as e:
        print(f"Unexpected error reading the file: {e}")
        return None

def open_file_dialog():
    """
    Opens a tkinter dialog to select a file.
    """
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window

    file_path = filedialog.askopenfilename(
        title="Select the route JSON file",
        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
    )
    
    root.destroy()  # It's good practice to destroy the root instance if not using mainloop

    if file_path:
        return load_json_from_path(file_path)
    return None

def parse_arguments():
    """
    Configure and parse command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="DARP Route Visualizer."
    )
    parser.add_argument(
        '-l', '--load',
        type=str,
        help='Path to the JSON file to load. If omitted, a file selector will open.',
        metavar='PATH'
    )
    return parser.parse_args()

def main():
    # 1. Process arguments
    args = parse_arguments()
    data = None

    # 2. Decide loading strategy
    if args.load:
        # CLI strategy: Load from argument
        data = load_json_from_path(args.load)
        if data is None:
            # If CLI loading fails, exit with error (do not open GUI to avoid confusion in scripts)
            sys.exit(1)
    else:
        # GUI strategy: Open dialog
        print("No file specified. Opening selector...")
        data = open_file_dialog()

    # 3. Execute visualization
    if data:
        try:
            city = data.get('metadata', {}).get('city', None)
            if city == "_Cordeau_":
                visualizer = CordeauRouteVisualizer(data)
            else:
                visualizer = RouteVisualizer(data)
            visualizer.plot()
        except Exception as e:
            print(f"Error initializing the visualizer: {e}")
    else:
        print("Operation cancelled or invalid file.")

if __name__ == "__main__":
    main()