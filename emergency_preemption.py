#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import traci
import sumolib
from collections import defaultdict
import argparse
import xml.etree.ElementTree as ET

# We need to import Python modules from the SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Constants for the simulation
DETECTION_RADIUS = 50.0  # meters
EMERGENCY_VEHICLE_TYPES = {
    'veh_ambulance': 1,           # Priority 1 (Highest)
    'veh_ambulance_urgent': 1,    # Same highest priority for urgent ambulance
    'veh_firefighter': 2,         # Priority 2 
    'veh_firefighter_heavy': 2,   # Same priority for heavy fire trucks
    'veh_police': 3,              # Priority 3
    'veh_police_swat': 3,         # Same priority for SWAT teams
    'veh_disaster_response': 2,   # Priority 2 for disaster response
    'veh_medical_transport': 4,   # Priority 4 for non-emergency medical
    'veh_hazmat': 2              # Priority 2 for hazardous materials response
}
NORMAL_VEHICLE_TYPE = 'veh_passenger'  # Priority 0
DEFAULT_SIM_TIME = 3600  # Default simulation time in seconds (1 hour)

# Global variables
junctions_with_tls = []
currently_preempted_tls = {}
preemption_memory = {}  # To track past preemptions and recover previous states

def get_approaching_lane(vehicle_id, tls_id):
    """Determine which lane the vehicle is approaching for the given traffic light"""
    route = traci.vehicle.getRoute(vehicle_id)
    route_index = traci.vehicle.getRouteIndex(vehicle_id)
    
    if route_index + 1 >= len(route):
        return None  # Vehicle at the end of its route
        
    current_edge = traci.vehicle.getRoadID(vehicle_id)
    next_edge = route[route_index + 1] if route_index < len(route) - 1 else None
    
    # Get controlled links for the traffic light
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
    
    # Check if any link's fromEdge matches the vehicle's current or next edge
    for i, links in enumerate(controlled_links):
        for link in links:
            if link[0].split('_')[0] == current_edge or (next_edge and link[0].split('_')[0] == next_edge):
                return i  # Return the link index
    
    return None

def find_nearest_tls(vehicle_id):
    """Find the nearest traffic light(s) to a vehicle"""
    nearby_tls = []
    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
    vehicle_edge = traci.vehicle.getRoadID(vehicle_id)
    
    # Skip if vehicle is on an internal edge (already in junction)
    if vehicle_edge.startswith(':'):
        return []
    
    for tls_id in traci.trafficlight.getIDList():
        tls_pos = traci.junction.getPosition(tls_id)
        distance = sumolib.geomhelper.distanceXY(vehicle_pos[0], vehicle_pos[1], tls_pos[0], tls_pos[1])
        
        if distance <= DETECTION_RADIUS:
            approaching_lane = get_approaching_lane(vehicle_id, tls_id)
            if approaching_lane is not None:
                nearby_tls.append((tls_id, distance, approaching_lane))
    
    # Sort by distance
    nearby_tls.sort(key=lambda x: x[1])
    return nearby_tls

def preempt_traffic_light(tls_id, approach_lane, vehicle_id, vehicle_priority):
    """Preempt a traffic light to give green to the specified approach lane"""
    if tls_id in currently_preempted_tls and currently_preempted_tls[tls_id][1] <= vehicle_priority:
        # Already preempted by a vehicle with equal or higher priority
        return False
    
    # Remember current state before preemption
    if tls_id not in preemption_memory:
        preemption_memory[tls_id] = {
            'program': traci.trafficlight.getProgram(tls_id),
            'phase': traci.trafficlight.getPhase(tls_id),
            'phase_duration': traci.trafficlight.getPhaseDuration(tls_id),
            'remaining_duration': traci.trafficlight.getNextSwitch(tls_id) - traci.simulation.getTime()
        }
    
    # Create custom state to set green for the approach lane
    current_state = list(traci.trafficlight.getRedYellowGreenState(tls_id))
    num_links = len(current_state)
    
    # Make all signals red
    new_state = ['r'] * num_links
    
    # Get the traffic light logic
    logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]  # Use first program logic
    
    # Find controlled links for the approaching lane
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
    
    # Set green for the approach lane and its compatible movements
    for i, links in enumerate(controlled_links):
        for link in links:
            if i == approach_lane or (len(link) > 2 and link[2]):  # If it's our lane or compatible
                new_state[i] = 'g'
    
    # Apply the new state
    traci.trafficlight.setRedYellowGreenState(tls_id, ''.join(new_state))
    
    # Store the preemption info
    currently_preempted_tls[tls_id] = (vehicle_id, vehicle_priority, approach_lane)
    
    print(f"Preempting TLS {tls_id} for {vehicle_id} (priority {vehicle_priority}) on approach {approach_lane}")
    return True

def restore_traffic_light(tls_id):
    """Restore traffic light to normal operation after preemption"""
    if tls_id in preemption_memory:
        mem = preemption_memory[tls_id]
        traci.trafficlight.setProgram(tls_id, mem['program'])
        traci.trafficlight.setPhase(tls_id, mem['phase'])
        traci.trafficlight.setPhaseDuration(tls_id, mem['remaining_duration'])
        del preemption_memory[tls_id]
        
    if tls_id in currently_preempted_tls:
        del currently_preempted_tls[tls_id]
        print(f"Restored TLS {tls_id} to normal operation")

def is_emergency_vehicle_in_junction(tls_id):
    """Check if the emergency vehicle that triggered preemption is still in the junction"""
    if tls_id not in currently_preempted_tls:
        return False
        
    vehicle_id = currently_preempted_tls[tls_id][0]
    
    if vehicle_id not in traci.vehicle.getIDList():
        # Vehicle has left the simulation
        return False
        
    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
    tls_pos = traci.junction.getPosition(tls_id)
    distance = sumolib.geomhelper.distanceXY(vehicle_pos[0], vehicle_pos[1], tls_pos[0], tls_pos[1])
    
    # Check if vehicle is still near the junction
    return distance <= DETECTION_RADIUS

def process_emergency_vehicles():
    """Process all emergency vehicles in the simulation"""
    emergency_vehicles = []
    
    # Identify all emergency vehicles in the simulation
    for veh_id in traci.vehicle.getIDList():
        veh_type = traci.vehicle.getTypeID(veh_id)
        if veh_type in EMERGENCY_VEHICLE_TYPES:
            priority = EMERGENCY_VEHICLE_TYPES[veh_type]
            emergency_vehicles.append((veh_id, priority))
            
            # Make emergency vehicles more visible by adding special color effects
            try:
                if veh_type.startswith('veh_ambulance'):
                    # Flashing red/white for ambulance
                    if step % 2 == 0:
                        traci.vehicle.setColor(veh_id, (255, 0, 0, 255))  # Red
                    else:
                        traci.vehicle.setColor(veh_id, (255, 255, 255, 255))  # White
                elif veh_type.startswith('veh_police'):
                    # Flashing blue/red for police
                    if step % 2 == 0:
                        traci.vehicle.setColor(veh_id, (0, 0, 255, 255))  # Blue
                    else:
                        traci.vehicle.setColor(veh_id, (255, 0, 0, 255))  # Red
                elif veh_type.startswith('veh_fire'):
                    # Flashing red/yellow for fire
                    if step % 2 == 0:
                        traci.vehicle.setColor(veh_id, (255, 0, 0, 255))  # Red
                    else:
                        traci.vehicle.setColor(veh_id, (255, 255, 0, 255))  # Yellow
            except:
                pass  # Ignore color setting errors
    
    # Sort by priority (lower number = higher priority)
    emergency_vehicles.sort(key=lambda x: x[1])
    
    # Process each emergency vehicle
    for veh_id, priority in emergency_vehicles:
        try:
            nearby_tls = find_nearest_tls(veh_id)
            
            for tls_id, distance, approach_lane in nearby_tls:
                preempt_traffic_light(tls_id, approach_lane, veh_id, priority)
        except traci.exceptions.TraCIException as e:
            print(f"Warning: Error processing emergency vehicle {veh_id}: {e}")
            continue

def check_preempted_junctions():
    """Check if preemption can be released for any junctions"""
    tls_to_restore = []
    
    for tls_id in list(currently_preempted_tls.keys()):
        if not is_emergency_vehicle_in_junction(tls_id):
            tls_to_restore.append(tls_id)
    
    for tls_id in tls_to_restore:
        restore_traffic_light(tls_id)

def fix_emergency_routes(sumo_cmd):
    """Create a temporary emergency routes file with valid edge IDs"""
    import xml.etree.ElementTree as ET
    import tempfile
    
    # Get all valid edges in the network
    net_file = None
    for i, arg in enumerate(sumo_cmd):
        if arg == "-n" or arg == "--net-file":
            if i+1 < len(sumo_cmd):
                net_file = sumo_cmd[i+1]
                break
    
    if not net_file:
        for arg in sumo_cmd:
            if arg.startswith("-c=") or arg.startswith("--configuration-file="):
                config_file = arg.split("=")[1]
                config = ET.parse(config_file)
                for input_elem in config.getroot().findall(".//input/net-file"):
                    net_file = input_elem.get("value")
                    break
    
    if not net_file:
        print("Warning: Could not find network file to check valid edges")
        return sumo_cmd
    
    # Check if the network file is a path or just a filename
    if not os.path.isabs(net_file):
        net_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), net_file)
    
    # Create a temporary file for the corrected routes
    original_route_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "emergency_routes.rou.xml")
    
    # Find a valid edge to replace the invalid one
    valid_edge = "420496529#1"  # Default fallback edge from the attachments
    
    try:
        # Try to load the network to get valid edges
        net = sumolib.net.readNet(net_file)
        all_edges = [edge.getID() for edge in net.getEdges()]
        if all_edges:
            valid_edge = all_edges[0]
    except Exception as e:
        print(f"Warning: Could not read network file: {e}")
    
    # Create a temporary file with corrected routes
    fd, temp_file = tempfile.mkstemp(suffix=".rou.xml", prefix="emergency_routes_fixed_")
    os.close(fd)
    
    try:
        tree = ET.parse(original_route_file)
        root = tree.getroot()
        
        # Replace all instances of the problematic edge
        for trip in root.findall(".//trip"):
            from_edge = trip.get("from")
            if from_edge == "964015634#0":
                print(f"Fixing route for {trip.get('id')}: replacing edge {from_edge} with {valid_edge}")
                trip.set("from", valid_edge)
        
        # Write the modified XML to the temporary file
        tree.write(temp_file)
        
        # Update the sumo command to use the new file
        for i, arg in enumerate(sumo_cmd):
            if "emergency_routes.rou.xml" in arg:
                sumo_cmd[i] = temp_file
                break
            elif arg.startswith("-r=") or arg.startswith("--route-files="):
                routes = arg.split("=")[1].split(",")
                new_routes = []
                for route in routes:
                    if "emergency_routes.rou.xml" in route:
                        new_routes.append(temp_file)
                    else:
                        new_routes.append(route)
                sumo_cmd[i] = f"{arg.split('=')[0]}={','.join(new_routes)}"
                break
        
        print(f"Created temporary file with fixed routes: {temp_file}")
    except Exception as e:
        print(f"Warning: Failed to fix emergency routes: {e}")
    
    return sumo_cmd

def fix_emergency_routes_permanently():
    """Permanently fix the emergency routes file by replacing invalid edges"""
    route_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "emergency_routes.rou.xml")
    if not os.path.exists(route_file):
        print(f"Error: Emergency routes file not found at {route_file}")
        return False
        
    # Create backup if it doesn't exist
    backup_file = route_file + ".bak"
    if not os.path.exists(backup_file):
        import shutil
        shutil.copy(route_file, backup_file)
        print(f"Created backup at {backup_file}")
    
    # Valid edge to use as replacement
    valid_edge = "420496529#1"  # Known valid edge from logs
    
    try:
        # Load the network to verify valid edges if possible
        net_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osm.net.xml.gz")
        if os.path.exists(net_file):
            try:
                net = sumolib.net.readNet(net_file)
                all_edges = [edge.getID() for edge in net.getEdges()]
                if all_edges:
                    valid_edge = all_edges[0]  # Use first edge as replacement
                    print(f"Using {valid_edge} as replacement edge")
            except Exception as e:
                print(f"Warning: Could not read network file: {e}")
        
        # Parse and fix the routes file
        tree = ET.parse(route_file)
        root = tree.getroot()
        fixes_applied = 0
        
        # Replace problematic edges
        problematic_edges = ["964015634#0"]
        
        for trip in root.findall(".//trip"):
            from_edge = trip.get("from")
            to_edge = trip.get("to")
            
            if from_edge in problematic_edges:
                print(f"Fixing route for {trip.get('id')}: replacing 'from' edge {from_edge} with {valid_edge}")
                trip.set("from", valid_edge)
                fixes_applied += 1
                
            if to_edge in problematic_edges:
                print(f"Fixing route for {trip.get('id')}: replacing 'to' edge {to_edge} with {valid_edge}")
                trip.set("to", valid_edge)
                fixes_applied += 1
        
        # Save the fixed file if changes were made
        if fixes_applied > 0:
            tree.write(route_file)
            print(f"Applied {fixes_applied} fixes to {route_file}")
            return True
        else:
            print("No fixes needed in emergency routes file")
            return False
    
    except Exception as e:
        print(f"Error fixing emergency routes file: {e}")
        return False

def run(min_sim_time=DEFAULT_SIM_TIME):
    """Run the simulation with emergency vehicle preemption"""
    step = 0
    
    # Fix the emergency routes file permanently before starting the simulation
    fix_emergency_routes_permanently()
    
    # Prepare SUMO command
    sumo_cmd = ["sumo-gui", "-c", "osm.sumocfg", "--start", "--quit-on-end"]
    
    # Still use the temporary fix as a backup
    sumo_cmd = fix_emergency_routes(sumo_cmd)
    
    try:
        # Start the TraCI connection
        traci.start(sumo_cmd)
        
        # Get the current simulation time
        sim_start_time = traci.simulation.getTime()
        min_end_time = sim_start_time + min_sim_time
        
        while True:
            current_time = traci.simulation.getTime()
            vehicles_left = traci.simulation.getMinExpectedNumber() > 0
            
            # Continue if there are vehicles or we haven't reached the minimum time
            if not vehicles_left and current_time >= min_end_time:
                break
                
            traci.simulationStep()
            
            try:
                # Process emergency vehicles and preempt traffic lights as needed
                process_emergency_vehicles()
                
                # Check if any preemption can be released
                check_preempted_junctions()
            except Exception as e:
                print(f"Error in simulation step {step}: {e}")
            
            step += 1
            
            # Print progress every 100 steps
            if step % 100 == 0:
                print(f"Simulation step: {step}, Time: {current_time:.2f}, Vehicles: {traci.vehicle.getIDCount()}")

    except traci.exceptions.FatalTraCIError as e:
        print(f"Fatal TraCI error: {e}")
    finally:
        if traci.isConnected():
            traci.close()
        print(f"Simulation completed at time: {traci.simulation.getTime():.2f}")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run SUMO simulation with emergency vehicle preemption")
    parser.add_argument("--time", type=int, default=DEFAULT_SIM_TIME,
                      help=f"Minimum simulation time in seconds (default: {DEFAULT_SIM_TIME})")
    args = parser.parse_args()
    
    run(min_sim_time=args.time)
