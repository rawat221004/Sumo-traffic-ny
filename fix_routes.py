#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as ET
import sumolib

# Path to the emergency routes file
routes_file = "emergency_routes.rou.xml"
# Path to the network file
net_file = "osm.net.xml.gz"

def fix_routes():
    # Make sure the files exist
    if not os.path.exists(routes_file):
        print(f"Error: {routes_file} not found!")
        return False
    
    if not os.path.exists(net_file):
        print(f"Error: {net_file} not found!")
        return False
    
    # Get valid edges from the network
    try:
        net = sumolib.net.readNet(net_file)
        valid_edges = [edge.getID() for edge in net.getEdges()]
        if not valid_edges:
            print("Warning: No valid edges found in the network!")
            valid_edge = "420496529#1"  # Fallback to a known edge from logs
        else:
            valid_edge = valid_edges[0]
            print(f"Found {len(valid_edges)} valid edges in the network")
    except Exception as e:
        print(f"Error reading network file: {e}")
        valid_edge = "420496529#1"  # Fallback to a known edge from logs
    
    print(f"Using {valid_edge} as a replacement for invalid edges")
    
    # Parse and fix the routes file
    try:
        tree = ET.parse(routes_file)
        root = tree.getroot()
        
        # Make a backup of the original file
        backup_file = routes_file + ".bak"
        if not os.path.exists(backup_file):
            tree.write(backup_file)
            print(f"Created backup at {backup_file}")
        
        # Count of fixed routes
        fixed_count = 0
        
        # Check all trips
        for trip in root.findall(".//trip"):
            from_edge = trip.get("from")
            to_edge = trip.get("to")
            
            # Check if 'from' edge is valid
            if from_edge not in valid_edges:
                trip_id = trip.get("id")
                print(f"Fixing route for {trip_id}: replacing 'from' edge {from_edge} with {valid_edge}")
                trip.set("from", valid_edge)
                fixed_count += 1
            
            # Check if 'to' edge is valid
            if to_edge not in valid_edges:
                trip_id = trip.get("id")
                print(f"Fixing route for {trip_id}: replacing 'to' edge {to_edge} with {valid_edge}")
                trip.set("to", valid_edge)
                fixed_count += 1
        
        # Save the fixed file
        tree.write(routes_file)
        print(f"Fixed {fixed_count} invalid edges in {routes_file}")
        return True
    
    except Exception as e:
        print(f"Error fixing routes file: {e}")
        return False

if __name__ == "__main__":
    if fix_routes():
        print("Routes fixed successfully! You can now run the simulation.")
    else:
        print("Failed to fix routes. Check the error messages above.")
