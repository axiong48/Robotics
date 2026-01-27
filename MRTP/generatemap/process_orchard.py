import cv2
import numpy as np
import math
import os
import random

# ================= CONFIGURATION =================
MAP_FILE = "orchard_map.png"
OUTPUT_FILE = "generated_orchard.sdf"

# 1. TREE VARIANTS (Your 5 random trees)
TREE_VARIANTS = [
    "model://orchard/orchard_tree_1.dae",
    "model://orchard/orchard_tree_2.dae",
    "model://orchard/orchard_tree_3.dae",
    "model://orchard/orchard_tree_4.dae",
    "model://orchard/orchard_tree_5.dae"
]
GROUND_URI = "model://orchard/orchard_world.dae" 

# 2. COLOR DETECTION (Wider range to catch everything)
LOWER_GREEN = np.array([25, 40, 20])   
UPPER_GREEN = np.array([100, 255, 255]) 

# 3. TUNING (THE IMPORTANT PART)
RESOLUTION = 0.1
GLOBAL_SCALE_MULTIPLIER = 1.0 
TRUNK_HEIGHT = 1.0  
TRUNK_RADIUS = 0.2

# Separation Sensitivity (0.1 to 1.0)
# Lower = Finds more trees (even slightly connected ones)
# Higher = Only finds very distinct peaks
SEPARATION_SENSITIVITY = 0.1
# =================================================

def generate_sdf():
    print(f"--- Starting Distance Transform Generator ---")
    
    if not os.path.exists(MAP_FILE):
        print("ERROR: Map file not found.")
        return

    # 1. Load Image & Mask
    img = cv2.imread(MAP_FILE)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # 2. Clean noise
    # Open (Erode->Dilate) removes white specks (noise) without shrinking trees too much
    kernel = np.ones((3,3), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    # 3. DISTANCE TRANSFORM (The Magic Fix)
    # This creates a height-map where the center of a tree is brightest
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    
    # Normalize for debugging visibility
    cv2.normalize(dist_transform, dist_transform, 0, 1.0, cv2.NORM_MINMAX)

    # 4. Find Peaks (Tree Centers)
    # We ignore the edges of the "worm" and only look at the peaks
    _, sure_fg = cv2.threshold(dist_transform, SEPARATION_SENSITIVITY * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    # Save debug images so you can see what happened!
    cv2.imwrite("debug_1_mask.png", mask)
    cv2.imwrite("debug_2_centers.png", sure_fg)
    print("Saved debug images. CHECK 'debug_2_centers.png' - these dots are your trees!")

    # 5. Find Contours on the CENTERS, not the blobs
    contours, _ = cv2.findContours(sure_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} trees after separation.")

    sdf = []
    sdf.append("<?xml version='1.0'?>")
    sdf.append("<sdf version='1.7'>")
    sdf.append("  <world name='procedural_orchard'>")
    sdf.append("    <physics type='ode'><max_step_size>0.001</max_step_size></physics>")
    sdf.append("    <include><uri>model://sun</uri></include>")

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
        # Filter noise
        if cv2.contourArea(contour) < 2.0: continue

        # Get Center of the "Peak"
        M = cv2.moments(contour)
        if M["m00"] == 0: continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # World Coordinates
        world_x = (cX - center_x) * RESOLUTION
        world_y = -(cY - center_y) * RESOLUTION
        
        # Randomize Rotation
        world_yaw = random.uniform(0, 6.28)

        # SCALING: Random Variation
        # Since we are using peaks, we can't measure the exact width of the original blob easily.
        # It is safer to use a base size + randomization.
        rand_scale = random.uniform(0.8, 1.2) 
        final_scale = rand_scale * GLOBAL_SCALE_MULTIPLIER
        
        # Pick Variant
        selected_tree_uri = random.choice(TREE_VARIANTS)

        sdf.append(f"""
    <model name='tree_{i}'>
      <static>true</static>
      <pose>{world_x:.3f} {world_y:.3f} 0 0 0 {world_yaw:.3f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><mesh>
            <uri>{selected_tree_uri}</uri>
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

    with open(OUTPUT_FILE, "w") as f:
        f.write("\n".join(sdf))
    
    print(f"--- Success! Generated {OUTPUT_FILE} ---")

if __name__ == "__main__":
    generate_sdf()