import os
import json
import random
import math

def euclidean_distance(x1, y1, x2, y2):
    return round(math.sqrt((x1 - x2)**2 + (y1 - y2)**2), 5)

def export_to_cordeau_text(instance_data, coords_map, output_filepath):
    """
    Exports the generated MDDARP instance into a space-separated text file,
    closely mimicking the Cordeau and Laporte (2003) format but extended 
    for heterogeneous vehicles and multiple depots.
    """
    n_veh = len(instance_data["vehicles"])
    n_req = instance_data["requests"]["n_requests"]
    l_ride = instance_data["global_params"]["L_ride"]
    
    with open(output_filepath, 'w') as f:
        # 1. Header: m, n, max_route_time, capacity, max_ride_time
        # Set max_route_time and capacity to 0 in header to denote heterogeneity
        f.write(f"{n_veh} {n_req} 0 0 {l_ride}\n")
        
        # 2. Heterogeneous Vehicles (Custom addition to Cordeau format)
        f.write("\n# VEHICLES: id start_node end_node capacity max_time\n")
        for v in instance_data["vehicles"]:
            f.write(f"{v['id']} {v['start_node']} {v['end_node']} {v['capacity']} {v['max_time']}\n")
            
        # 3. Nodes: id x y service_time demand tw_start tw_end
        f.write("\n# NODES: id x y service_time demand tw_start tw_end\n")
        for node in instance_data["nodes"]:
            n_id = node["id"]
            
            # Retrieve coordinates
            x = coords_map[n_id]["x"]
            y = coords_map[n_id]["y"]
            
            # Format to strictly match the requested parameters
            line = f"{n_id} {x:.4f} {y:.4f} {node['service_time']} {node['demand']} {node['tw_start']} {node['tw_end']}\n"
            f.write(line)

def generate_mddarp_instances(num_instances=20, output_dir="./mddarp-inst/", seed=123):
    random.seed(seed)
    os.makedirs(output_dir, exist_ok=True)
    
    # Instance sizes similar to Cordeau & Laporte 2003
    sizes = [
        (24, 3), (48, 5), (72, 7), (96, 9), (120, 11), (144, 13),
        (36, 4), (72, 6), (108, 8), (144, 10)
    ]
    
    for inst_idx in range(1, num_instances + 1):
        if inst_idx <= len(sizes):
            n_req, n_vehicles = sizes[inst_idx - 1]
        else:
            n_req = random.choice([24, 48, 72, 96, 120])
            n_vehicles = max(3, n_req // 10)
            
        L_ride = 90
        planning_horizon = 600
        service_time = 10
        
        # 1. Generate Coordinates for requests
        coords = {}
        for i in range(1, (2 * n_req) + 1):
            coords[i] = {
                "x": round(random.uniform(-10.0, 10.0), 4),
                "y": round(random.uniform(-10.0, 10.0), 4)
            }
            
        # 2. Generate Heterogeneous Vehicles & Depots
        vehicles = []
        depot_coords = {}
        for k in range(1, n_vehicles + 1):
            start_node_id = (2 * n_req) + k
            end_node_id = (2 * n_req) + n_vehicles + k
            
            # Heterogeneous properties
            capacity = random.choice([4, 6, 8])
            
            # Heterogeneous operational shifts (constrained to guarantee global feasibility)
            shift_start = random.randint(0, 60) # Ensures vehicles are awake for early requests
            shift_end = random.randint(540, planning_horizon) # Ensures coverage later in the day
            
            # Generate and Clamp Max Route Time to ensure feasibility
            raw_max_time = random.randint(420, 600)
            clamped_max_time = min(raw_max_time, shift_end - shift_start)
            
            # Distinct depots for each vehicle start/end
            depot_coords[start_node_id] = {
                "x": round(random.uniform(-10.0, 10.0), 4),
                "y": round(random.uniform(-10.0, 10.0), 4)
            }
            depot_coords[end_node_id] = {
                "x": round(random.uniform(-10.0, 10.0), 4),
                "y": round(random.uniform(-10.0, 10.0), 4)
            }
            
            vehicles.append({
                "id": k,
                "start_node": start_node_id,
                "end_node": end_node_id,
                "capacity": capacity,
                "max_time": clamped_max_time,
                "shift_start": shift_start,
                "shift_end": shift_end
            })

        # 3. Build Nodes List
        nodes = []
        metadata_coords = {}
        pickup_ids = []
        delivery_ids = []
        
        half_n = n_req // 2  # Used for Outbound/Inbound split
        
        for i in range(1, n_req + 1):
            pickup_ids.append(i)
            delivery_ids.append(i + n_req)
            
            # Record coordinates in metadata
            metadata_coords[str(i)] = [coords[i]["y"], coords[i]["x"]]
            metadata_coords[str(i + n_req)] = [coords[i + n_req]["y"], coords[i + n_req]["x"]]

            if i <= half_n:
                # OUTBOUND REQUEST: Strict Pickup, Relaxed Delivery
                e_pickup = random.randint(60, 360) # Scaled for T=600
                l_pickup = e_pickup + random.randint(15, 45)
                
                nodes.append({
                    "id": i, "service_time": service_time, "demand": 1,
                    "tw_start": e_pickup, "tw_end": l_pickup
                })
                nodes.append({
                    "id": i + n_req, "service_time": service_time, "demand": -1,
                    "tw_start": 0, "tw_end": planning_horizon
                })
            else:
                # INBOUND REQUEST: Relaxed Pickup, Strict Delivery
                e_delivery = random.randint(180, 480) # Placed later in the day, scaled for T=600
                l_delivery = e_delivery + random.randint(15, 45)
                
                nodes.append({
                    "id": i, "service_time": service_time, "demand": 1,
                    "tw_start": 0, "tw_end": planning_horizon
                })
                nodes.append({
                    "id": i + n_req, "service_time": service_time, "demand": -1,
                    "tw_start": e_delivery, "tw_end": l_delivery
                })

        # Add Start/End Depot Nodes to Node List using Clamped Time Windows
        for v in vehicles:
            s_id = v["start_node"]
            e_id = v["end_node"]
            
            nodes.append({
                "id": s_id, "service_time": 0, "demand": 0,
                "tw_start": v["shift_start"], "tw_end": v["shift_end"]
            })
            metadata_coords[str(s_id)] = [depot_coords[s_id]["y"], depot_coords[s_id]["x"]]
            coords[s_id] = depot_coords[s_id]
            
            nodes.append({
                "id": e_id, "service_time": 0, "demand": 0,
                "tw_start": v["shift_start"], "tw_end": v["shift_end"]
            })
            metadata_coords[str(e_id)] = [depot_coords[e_id]["y"], depot_coords[e_id]["x"]]
            coords[e_id] = depot_coords[e_id]
            
            # Clean up temporary shift keys
            del v["shift_start"]
            del v["shift_end"]

        # 4. Generate Distance Matrices
        matrix_t = []
        matrix_c = []
        all_node_ids = list(coords.keys())
        
        for i in all_node_ids:
            for j in all_node_ids:
                dist = euclidean_distance(coords[i]["x"], coords[i]["y"], coords[j]["x"], coords[j]["y"])
                matrix_t.append({"from": i, "to": j, "value": dist})
                for v in vehicles:
                    matrix_c.append({
                        "k": v["id"],
                        "from": i,
                        "to": j,
                        "value": dist
                    })

        # 5. Compile Output
        output_data = {
            "global_params": {"L_ride": L_ride},
            "requests": {
                "n_requests": n_req,
                "pickup_ids": pickup_ids,
                "delivery_ids": delivery_ids
            },
            "vehicles": vehicles,
            "nodes": nodes,
            "matrix_t": matrix_t,
            "matrix_c": matrix_c,
            "metadata": {
                "city": f"_Cordeau_MDH_Inst_{inst_idx}_",
                "coordinates": metadata_coords
            }
        }
        
        # 6. Save JSON
        json_file_path = os.path.join(output_dir, f"md{inst_idx:02d}.json")
        with open(json_file_path, 'w') as f:
            json.dump(output_data, f, indent=2)

        # 7. Save Cordeau-like Text Format
        text_file_path = os.path.join(output_dir, f"md{inst_idx:02d}.txt")
        export_to_cordeau_text(output_data, coords, text_file_path)

        print(f"Instance {inst_idx} generated: {n_req} requests, {n_vehicles} vehicles.")
            
    print(f"Successfully generated {num_instances} strictly feasible MDDARP instances in '{output_dir}'.")

if __name__ == "__main__":
    generate_mddarp_instances(num_instances=20, seed=123)