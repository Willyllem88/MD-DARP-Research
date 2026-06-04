"""
MD-DARP Launcher
Entry point for the integrated MD-DARP workflow application.
"""

import tkinter as tk
from launcher_app import DARPLauncherApp


if __name__ == "__main__":
    root = tk.Tk()
    app = DARPLauncherApp(root)
    root.mainloop()
