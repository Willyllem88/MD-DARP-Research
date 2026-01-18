# DARP Solution Visualizer

A lightweight tool to visualize the routes and schedules generated for the Dial-a-Ride Multi-Depot Problem (DARP-MD) from JSON instance files.

## Installation

1. Ensure you have Python installed (3.10+ recommended).
2. Install the required dependencies:

```bash
pip install -r requirements.txt

```

*Note for Linux users: You may need to install `python3-tk` via your package manager if it's not already installed. In Ubuntu/Debian, you can do this with:*

```bash
sudo apt-get install python3-tk

```

## How to Run

You can run the visualizer in two modes: **Interactive (GUI)** or **Command Line (CLI)**.

### 1. Interactive Mode (File Picker)

If you run the script without arguments, a system file dialog will open, allowing you to browse and select the `.json` file manually.

```bash
python viewer.py

```

### 2. Command Line Mode (Direct Load)

You can specify the path to the JSON file directly using the `-l` (or `--load`) flag. This is efficient for quick checks or automated scripts.

```bash
python viewer.py -l results/my_instance.json

```