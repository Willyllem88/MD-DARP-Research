# Interactive DARP-MD Instance Generator

A graphical tool to generate instances for the Dial-a-Ride Problem (Multi-Depot) using real-world map data from OpenStreetMap.

## Installation

1. Ensure you have Python installed (3.10+ recommended).
2. Install the required dependencies:

```bash
pip install -r ../requirements.txt

```

*Note for Linux users: You may need to install `python3-tk` via your package manager if it's not already installed. In Ubuntu/Debian, you can do this with:*

```bash
sudo apt-get install python3-tk
```

## How to Run

Run the main script to start the application:

```bash
python main.py

```

## How to Use

1. **Configuration:**

    * Select a city/place from the dropdown (or type one manually).
    * Set the number of **Requests** and **Vehicles**.
    * Click **Load Map**.

2. **Selection (Point & Click):**

    * The status bar will guide you.
    * Click on the map to set **Pickup** and **Delivery** locations for each request.
    * Click on the map to set **Start** and **End** depots for each vehicle.
    * Pop-ups will appear asking for specific constraints (Time Windows, Load, etc.).

3. **Generation:**

    * Once all points are selected, the tool automatically calculates the travel time/cost matrices.
    * A file dialog will open to save the result as a `.json` file.
