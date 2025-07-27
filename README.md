# Obstacle Detection Bot

This project combines an Arduino-based two-wheel differential robot with ROS 2 Humble and Gazebo Classic simulation for robust obstacle detection and avoidance. Below you’ll find:

- **Software prerequisites**  
- **Hardware requirements**
- **Repository layout**  
- **Setup & build instructions**  
- **How to run the Gazebo simulation**  
- **How to deploy on real hardware**  
- **Troubleshooting tips**  

---

## 📋 Prerequisites

### Software

1. **Operating System**  
   - Windows 10 (2004+) or Windows 11 with WSL 2 support  
   - Ubuntu 22.04 under WSL 2  
2. **IDE & Extensions**  
   - Visual Studio Code on Windows  
   - **Remote – WSL** extension  
3. **ROS 2 & Gazebo**  
   - ROS 2 Humble Hawksbill (`ros-humble-desktop`)  
   - Gazebo 11 (`gazebo`, `libgazebo-dev`)  
   - ROS 2–Gazebo bridge packages (`ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-plugins`)  
4. **Build Tools**  
   - Python 3 & pip  
   - `python3-colcon-common-extensions`  

### Hardware
 
**Core components you’ll need:**

- **Microcontroller:** Arduino Nano (or Uno)  
- **Motor driver:** L293D (or L298N) dual H-bridge  
- **Drive:**  
  - 2 × DC motors + wheels  
  - Differential‐drive chassis (3D-printed or laser-cut)  
- **Sensors:**  
  - 3 × IR line-tracking sensors (e.g. TCRT5000)  
  - 1 × HC-SR04 ultrasonic distance sensor  
- **Power & wiring:**  
  - 6 × AA or 7.4 V LiPo battery pack  
  - Jumper wires & breadboard  
- **Extras (optional):** IMU, LiDAR, LiPo charger, enclosure  

---

## 📂 Repository Layout
obstacle-detection-bot/
├── CMakeLists.txt
├── package.xml
├── launch/ # ROS 2 launch files
├── urdf/ # Robot description (XACRO/URDF)
├── world/ # Gazebo worlds
├── model/ # Gazebo models
├── rviz/ # RViz configurations
├── script/ # Python nodes:
│ ├── joystick_control.py
│ ├── point_follow.py
│ ├── multipoint_follow.py
│ └── ultrasonic_sensor_obstacle_avoidance.py
├── config/ # Nav2 / SLAM params
├── img/ # Project screenshots & diagrams
└── README.md # ← you are here

## Install ROS and Gazebo deps
sudo apt update
sudo apt install -y \
  curl gnupg lsb-release \
  ros-humble-desktop \
  gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  python3-colcon-common-extensions

## Build the workspace
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
colcon build --merge-install

# Overlay
source install/setup.bash

## ▶️ Running the Gazebo Simulation
# Launch Gazebo + robot
ros2 launch rav_bot rav_gazebo.launch.py
You should see the small_warehouse world and your robot spawn.

# Start the obstacle-avoidance node
ros2 run rav_bot ultrasonic_sensor_obstacle_avoidance.py \
  --ros-args --remap /cmd_vel:=/rav_bot/cmd_vel
The node will process simulated /scan (or /ultrasonic) and publish to /rav_bot/cmd_vel.

# Visualize in RViz (optional)
ros2 launch rav_bot rviz.launch.py
Inspect TF frames, robot model, and sensor topics.

## 🚀 Deploying on Real Hardware
Assemble your chassis with Arduino Nano, motors, motor driver, IR sensors, and ultrasonic sensor.

Flash firmware to Arduino using rosserial or micro-ROS:

# Example with rosserial_python
ros2 run rosserial_python serial_node --dev /dev/ttyACM0
Power on and confirm obstacle avoidance in your environment.
