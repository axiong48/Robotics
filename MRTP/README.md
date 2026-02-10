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

Instructions [UPDATED 2/10]:

Run the following Node:

     ros2 run generatemap tcp_demux_node --ros-args -p output_path:="/MRTP/MRTP/clearpath_simulator/clearpath_gz/worlds/generated_orchard.sdf"


Node should now be listening to TCP port 12346 so run the bin: 

     nc localhost 12346 < create_test.bin

SDF file output will be in the worlds folder:

     cd..
     colcon build
     ros2 launch clearpath_gz simulation.launch.py world:=generated_orchard



