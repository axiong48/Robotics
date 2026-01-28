# Generate Orchard Simulation for ROS 2

This repository contains tools to procedurally generate 3D orchard environments for Gazebo based on 2D map images. **ROS 2 Jazzy**.

The system reads a map image, detects tree rows using computer vision, and generates a `.sdf` world file populated with random 3D tree models.

Ensure you have the following installed:
* **ROS 2 Jazzy**
* **Gazebo**
* **Python 3**


```bash
pip3 install opencv-python numpy
```

Cloning:

     git clone https://github.com/axiong48/Robotics
     
move to the folder MRTP/MRTP, and run

     colcon build

Instructions:

The lat and lon are determined to be in the center of the map.
move to the generatemap folder and run the following:

     python3 generate_orchard.py [name_of_map].png --lat [Y] --lon - [X]


a sdf file should appear in the worlds now you may: 


     colcon build



     ros2 launch clearpath_gz simulation.launch.py world:=generated_orchard
