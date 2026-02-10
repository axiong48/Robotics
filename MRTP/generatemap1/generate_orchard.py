import cv2
import numpy as np
import math
import os
import random
import argparse 

# 4 different tree models
TREE_MODELS = [
    "model://orchard/orchard_tree_1.dae",
    # "model://orchard/orchard_tree_2.dae",
    "model://orchard/orchard_tree_3.dae",
    "model://orchard/orchard_tree_4.dae",
    "model://orchard/orchard_tree_5.dae"
]

GROUND_URI = "model://orchard/orchard_world.dae" 

# you can tune this to make it more accurate depending on the 2D map
LOWER_GREEN = np.array([25, 40, 20])  
UPPER_GREEN = np.array([100, 255, 255])
TRUNK_HEIGHT = 1.0 
TRUNK_RADIUS = 0.2
BRIDGE_BREAKER_STRENGTH = 5
SEPARATION_SENSITIVITY = 0.35

def generate_sdf(image_path, output_path, lat, lon, elev, resolution, target_tree_width, base_model_width):
    # ensures the output directory exists
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        print(f"Creating output directory: {output_dir}")
        os.makedirs(output_dir)

    print(f"--- Generating Uniform World ---")
    print(f"Map: {image_path}")
    print(f"Target Tree Size: ~{target_tree_width} meters")
    
    if not os.path.exists(image_path):
        print(f"ERROR: File '{image_path}' not found.")
        return

    # Load & Process
    img = cv2.imread(image_path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    
    # Bridge Breaker
    kernel = np.ones((3,3), np.uint8)
    eroded_mask = cv2.erode(mask, kernel, iterations=BRIDGE_BREAKER_STRENGTH)
    
    # Find Centers of tree
    dist_transform = cv2.distanceTransform(eroded_mask, cv2.DIST_L2, 5)
    cv2.normalize(dist_transform, dist_transform, 0, 1.0, cv2.NORM_MINMAX)
    
    _, sure_fg = cv2.threshold(dist_transform, SEPARATION_SENSITIVITY * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)
    
    # debug image 
    cv2.imwrite("debug_detection.png", sure_fg)
    print(f"Saved debug view to 'debug_detection.png'")
    
    contours, _ = cv2.findContours(sure_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Detected {len(contours)} trees.")

    # 4. Generate SDF script
    sdf = []
    sdf.append("<?xml version='1.0' encoding='ASCII'?>")
    sdf.append("<sdf version='1.4'>")
    sdf.append("  <world name='generated_orchard'>")

    sdf.append(f"""
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elev}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    """)

    # from orchared.sdf
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

    center_x = img.shape[1] / 2.0
    center_y = img.shape[0] / 2.0
    
    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < 2.0: continue
        M = cv2.moments(contour)
        if M["m00"] == 0: continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        world_x = (cX - center_x) * resolution
        world_y = -(cY - center_y) * resolution
        world_yaw = random.uniform(0, 6.28)
        
        # Scaling Logic for trees
        base_scale = target_tree_width / base_model_width
        random_factor = random.uniform(1.0, 1.50)
        final_scale = base_scale * random_factor

        # Randomly pick one of the 5 models
        selected_model = random.choice(TREE_MODELS)

        sdf.append(f"""
    <model name='tree_{i}'>
      <static>true</static>
      <pose>{world_x:.3f} {world_y:.3f} 0 0 0 {world_yaw:.3f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><mesh>
            <uri>{selected_model}</uri>
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

    # csv_filename = "tree_coordinates.csv"
    # with open(csv_filename, "w") as f:
    #     f.write("tree_id,x_local,y_local,latitude,longitude\n")
    #     f.write("\n".join(tree_data_list))
    
    # print(f"--- Success! ---")
    # print(f"1. World File: {output_path}")
    # print(f"2. GPS Data:   {csv_filename} (Contains lat/lon for every tree)")
    
    print(f"--- Success! Saved world to {output_path} ---")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a uniform Gazebo world from an orchard map.")
    
    parser.add_argument("image", help="Path to the map image file")
    parser.add_argument("--lat", type=float, required=True, help="Latitude of the map center")
    parser.add_argument("--lon", type=float, required=True, help="Longitude of the map center")
    parser.add_argument("--elev", type=float, default=0.0, help="Elevation in meters")
    parser.add_argument("--res", type=float, default=0.1, help="Meters per pixel (default: 0.1)")
    
    parser.add_argument("--width", type=float, default=3.5, help="Target width of the real trees in meters")
    parser.add_argument("--model_width", type=float, default=3.0, help="Base width of the 3D model file")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    default_out_path = os.path.join(
        script_dir, 
        "../clearpath_simulator/clearpath_gz/worlds/generated_orchard.sdf"
    )
    
    parser.add_argument("--out", default=default_out_path, help="Output filename")

    args = parser.parse_args()

    final_output_path = os.path.abspath(args.out)

    generate_sdf(args.image, final_output_path, args.lat, args.lon, args.elev, args.res, args.width, args.model_width)
