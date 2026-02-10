import socket
import struct
import json
import os
import random
import argparse
import xml.etree.ElementTree as ET

# ================= DEFAULTS =================
DEFAULT_TREE_MODEL = "model://orchard/orchard_tree_1.dae"
GROUND_URI = "model://orchard/orchard_world.dae" 

# TUNING CONSTANTS
TRUNK_HEIGHT = 1.0 
TRUNK_RADIUS = 0.2
# Earth radius in meters for lat/lon conversion (approximate)
EARTH_RADIUS = 6378137.0 

def latlon_to_meters(lat, lon, anchor_lat, anchor_lon):
    """
    Converts a GPS coordinate to x/y meters relative to an anchor point.
    Uses a simple equirectangular projection which is sufficient for small areas.
    """
    d_lat = lat - anchor_lat
    d_lon = lon - anchor_lon
    
    # Convert degrees to radians
    r_lat = anchor_lat * math.pi / 180.0
    
    # Calculate y (North-South distance)
    y = d_lat * (math.pi / 180.0) * EARTH_RADIUS
    
    # Calculate x (East-West distance)
    x = d_lon * (math.pi / 180.0) * EARTH_RADIUS * math.cos(r_lat)
    
    return x, y

def generate_sdf_from_json(tree_data, output_path, target_tree_width, base_model_width):
    print(f"--- Generating World from JSON Data ---")
    
    if not tree_data:
        print("ERROR: No tree data found.")
        return

    # 1. Determine Anchor Point (Use the first tree or calculate a center)
    # We'll use the first tree as the 0,0 reference for local coordinates, 
    # but Gazebo's spherical_coordinates will handle the global position.
    # Actually, better to just pick the min lat/lon or the first tree as anchor.
    # Let's use the first tree to keep it simple, or calculate a bounding box center.
    lats = [t['lat'] for t in tree_data]
    lons = [t['lon'] for t in tree_data]
    
    anchor_lat = sum(lats) / len(lats)
    anchor_lon = sum(lons) / len(lons)
    
    print(f"Calculated Anchor Point: {anchor_lat}, {anchor_lon}")
    print(f"Processing {len(tree_data)} trees...")

    # 2. Generate SDF
    sdf = []
    sdf.append("<?xml version='1.0' encoding='ASCII'?>")
    sdf.append("<sdf version='1.4'>")
    sdf.append("  <world name='procedural_orchard'>")

    sdf.append(f"""
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{anchor_lat}</latitude_deg>
      <longitude_deg>{anchor_lon}</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    """)

    # Boilerplate Physics/Light
    sdf.append("""
    <physics type='ode'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='libgz-sim-physics-system.so'/>
    <plugin name='gz::sim::systems::UserCommands' filename='libgz-sim-user-commands-system.so'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='libgz-sim-scene-broadcaster-system.so'/>
    <plugin name='gz::sim::systems::Sensors' filename='libgz-sim-sensors-system.so'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='gz::sim::systems::Imu' filename='libgz-sim-imu-system.so'/>
    <plugin name='gz::sim::systems::NavSat' filename='libgz-sim-navsat-system.so'/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>1</shadows>
      <sky><clouds><speed>12</speed></clouds></sky>
    </scene>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1.0</direction>
    </light>
    """)

    sdf.append(f"""
    <model name='orchard_ground'>
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <collision name='col'><geometry><mesh><uri>{GROUND_URI}</uri></mesh></geometry></collision>
        <visual name='vis'><geometry><mesh><uri>{GROUND_URI}</uri></mesh></geometry></visual>
      </link>
    </model>
    """)

    for tree in tree_data:
        idx = tree['tree_index']
        lat = tree['lat']
        lon = tree['lon']
        
        # Calculate local x,y relative to anchor
        world_x, world_y = latlon_to_meters(lat, lon, anchor_lat, anchor_lon)
        world_yaw = random.uniform(0, 6.28)
        
        # Scale logic
        base_scale = target_tree_width / base_model_width
        random_factor = random.uniform(0.85, 1.15)
        final_scale = base_scale * random_factor

        sdf.append(f"""
    <model name='tree_{idx}'>
      <static>true</static>
      <pose>{world_x:.3f} {world_y:.3f} 0 0 0 {world_yaw:.3f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><mesh>
            <uri>{DEFAULT_TREE_MODEL}</uri>
            <scale>{final_scale:.3f} {final_scale:.3f} {final_scale:.3f}</scale>
          </mesh></geometry>
        </visual>
        <collision name='trunk_collision'>
          <pose>0 0 {TRUNK_HEIGHT/2} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{TRUNK_RADIUS}</radius>
              <length>{TRUNK_HEIGHT}</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>""")

    sdf.append("  </world>")
    sdf.append("</sdf>")

    with open(output_path, "w") as f:
        f.write("\n".join(sdf))
    
    print(f"--- Success! Saved world to {output_path} ---")

def start_tcp_server(port, output_path, width, model_width):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    
    print(f"Listening on port {port} for mission data...")
    print("Use command: nc localhost 12346 < your_data.bin")

    conn, addr = server_socket.accept()
    print(f"Connection from {addr}")

    try:
        # Based on C++ code, format is: 
        # [4-byte size][XML Payload][4-byte size][JSON Payload]
        # Assuming 'order' is xml_then_json and 'payload_length_included' is true
        
        # 1. Read XML Size
        size_data = conn.recv(4)
        if not size_data: return
        xml_size = struct.unpack('!I', size_data)[0] # Network byte order (Big Endian)
        
        # 2. Read XML
        xml_data = b''
        while len(xml_data) < xml_size:
            chunk = conn.recv(min(4096, xml_size - len(xml_data)))
            if not chunk: break
            xml_data += chunk
            
        print(f"Received XML ({len(xml_data)} bytes)")
        
        # 3. Read JSON Size
        size_data = conn.recv(4)
        if not size_data: return
        json_size = struct.unpack('!I', size_data)[0]
        
        # 4. Read JSON
        json_data = b''
        while len(json_data) < json_size:
            chunk = conn.recv(min(4096, json_size - len(json_data)))
            if not chunk: break
            json_data += chunk
            
        print(f"Received JSON ({len(json_data)} bytes)")
        
        # 5. Process JSON
        try:
            tree_list = json.loads(json_data.decode('utf-8'))
            generate_sdf_from_json(tree_list, output_path, width, model_width)
        except json.JSONDecodeError as e:
            print(f"Failed to decode JSON: {e}")

    finally:
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate Gazebo world from TCP stream.")
    parser.add_argument("--port", type=int, default=12346, help="Port to listen on")
    parser.add_argument("--out", default="generated_orchard.sdf", help="Output filename")
    parser.add_argument("--width", type=float, default=3.5, help="Target tree width (m)")
    parser.add_argument("--model_width", type=float, default=3.0, help="Base model width (m)")

    args = parser.parse_args()
    
    start_tcp_server(args.port, args.out, args.width, args.model_width)