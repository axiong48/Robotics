# MRTP: Amiga & Clearpath Map Generation

This repository serves as a complete ROS 2 development environment for the **Amiga** farm robot platform, integrated with the **Clearpath Simulator**. It features a custom map generation pipeline designed for precision navigation in orchard environments.

This shorthand feature introduced the amiga nodes in nav2 and generates an SDF to simulate a real world application to orchard gps locations.

---

## 📋 Prerequisites

Before setting up the workspace, ensure you have the following installed:

* **ROS 2 Jazzy** 
* **Gazebo**
* **Docker**
* **Netcat (`nc`)**: Used for binary data transmission to the demux node.
* **Python Dependencies**:
    ```bash
    pip install numpy setuptools
    ```

---

## 🚀 Installation & Setup

Ensure
- make vnc
- make build-image

### 1. Clone the Workspace
Clone the main repository and pull the Clearpath simulator dependency:
```bash
git clone https://github.com/axiong48/Robotics
cd Robotics
git submodule update --init --recursive
```

2. Build and Source
Compile the packages and update your environment variables:
```docker
cd MRTP
colcon build
source install/setup.bash
```

Running the Simulation
Follow these steps in order:

Step 1 Terminal 1: Start the TCP Demux Node
This node handles the sdf generation and outputs it in the correct folder:
```docker
ros2 run generatemap tcp_demux_node --ros-args -p output_path:="/MRTP/MRTP/clearpath_simulator/clearpath_gz/worlds/generated_orchard.sdf"
```

Step 2 Terminal 2: Feed the Map Data
Navigate to the map generator and send the binary test data via Netcat:
```docker
cd generatemap
nc localhost 12346 < create_test.bin
```

Step 3 Terminal 2: Launch Simulation Bringup
Return to the workspace root, rebuild, and launch:
```docker
cd ..
colcon build
ros2 launch generatemap bringup_simulation.launch.py
```
