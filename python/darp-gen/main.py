import tkinter as tk
import argparse, sys
from app import DarpApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", default=None,
                        help="Output JSON path (skips save dialog if provided)")
    args, _ = parser.parse_known_args()

    output_path = args.output

    root = tk.Tk()
    app = DarpApp(root, output_path=output_path)
    root.mainloop()