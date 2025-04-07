#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import traci
import sumolib
from collections import defaultdict

# We need to import Python modules from the SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Constants for the simulation
DETECTION_RADIUS = 50.0  # meters
EMERGENCY_VEHICLE_TYPES = {
    'veh_ambulance': 1,  # Priority 1 (Highest)
    'veh_firefighter': 2,  # Priority 2
    'veh_police': 3,  # Priority 3
}
NORMAL_VEHICLE_TYPE = 'veh_passenger'  # Priority 0

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
    
    # Sort by priority (lower number = higher priority)
    emergency_vehicles.sort(key=lambda x: x[1])
    
    # Process each emergency vehicle
    for veh_id, priority in emergency_vehicles:
        nearby_tls = find_nearest_tls(veh_id)
        
        for tls_id, distance, approach_lane in nearby_tls:
            preempt_traffic_light(tls_id, approach_lane, veh_id, priority)

def check_preempted_junctions():
    """Check if preemption can be released for any junctions"""
    tls_to_restore = []
    
    for tls_id in list(currently_preempted_tls.keys()):
        if not is_emergency_vehicle_in_junction(tls_id):
            tls_to_restore.append(tls_id)
    
    for tls_id in tls_to_restore:
        restore_traffic_light(tls_id)

def run():
    """Run the simulation with emergency vehicle preemption"""
    step = 0
    
    # Start the TraCI connection
    traci.start(["sumo-gui", "-c", "osm.sumocfg", "--start", "--quit-on-end"])
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        # Process emergency vehicles and preempt traffic lights as needed
        process_emergency_vehicles()
        
        # Check if any preemption can be released
        check_preempted_junctions()
        
        step += 1

    traci.close()
    print("Simulation completed - all vehicles have arrived at their destinations")

if __name__ == "__main__":
    run()
