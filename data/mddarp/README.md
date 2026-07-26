# Multi-Depot Dial-a-Ride Problem (MD-DARP) Instances

This dataset contains standard compressed benchmark instances (`.txt`) for the Multi-Depot Dial-a-Ride Problem (MD-DARP). The format extends the classic Cordeau and Laporte (2003) DARP format to support heterogeneous vehicle fleets and multi-depot configurations.

All spatial coordinates ($x, y$) in the text files are formatted as floating-point numbers **rounded to 5 decimal places**.

## File Structure (`.txt`)

Each `.txt` file is space-delimited and structured into three primary components:

### 1. Global Header

The first line defines the global setup of the instance:

```text
<n_vehicles> <n_requests> <max_ride_time>

```

* **`n_vehicles`**: Total number of vehicles available.
* **`n_requests`**: Total number of passenger transportation requests ($n$).
* **`max_ride_time`**: Global maximum ride duration allowed for any passenger.

### 2. Vehicle Fleet Block

Defines individual vehicle characteristics and depot assignments:

```text
# VEHICLES: id start_node end_node capacity max_time

```

* **`id`**: Unique integer ID of the vehicle.
* **`start_node`**: Node ID representing the vehicle's origin depot.
* **`end_node`**: Node ID representing the vehicle's destination depot.
* **`capacity`**: Maximum passenger capacity of the specific vehicle.
* **`max_time`**: Maximum route duration permitted for this vehicle.

### 3. Node Characteristics Block

Lists spatial and temporal parameters for all nodes (pickups, deliveries, and depots):

```text
# NODES: id x y service_time demand tw_start tw_end

```

* **`id`**: Node identifier.
  * $1 \dots n$: Pickup locations.
  * $n{+}1 \dots 2n$: Corresponding delivery locations for requests $1 \dots n$.
  * $> 2n$: Start and end depot locations for vehicles.
* **`x`**, **`y`**: Spatial coordinates (rounded strictly to **5 decimal places**).
* **`service_time`**: Time required to service the node (e.g., loading/unloading time).
* **`demand`**: Capacity variation (`+1` for pickups, `-1` for deliveries, `0` for depots).
* **`tw_start`**: Start of the node's service time window.
* **`tw_end`**: End of the node's service time window.