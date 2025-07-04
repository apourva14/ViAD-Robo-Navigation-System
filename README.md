# Obstacle Detection Bot

This guide will walk you through setting up the **obstacle-detection-bot** ROS 2 project on **Windows 10/11** using **WSL2 (Ubuntu 22.04)** and **VS Code** with the **Remote – WSL** extension.

---

## 📋 Prerequisites

1. **Windows 10 (2004+)** or **Windows 11** with WSL2 support
2. **VS Code** installed on Windows
3. **Administrator** access on your Windows machine

---

## 🔧 Step 1: WSL2 & Ubuntu 22.04 Setup

1. Open **PowerShell as Administrator** (Win + X → Windows PowerShell (Admin) or Start → type "PowerShell" → Ctrl+Shift+Enter).
2. Enable WSL & Virtual Machine Platform:

   ```powershell
   Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart
   Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart
   ```
3. Reboot Windows (if prompted).
4. In the **same** elevated PowerShell, set WSL2 as default and install Ubuntu:

   ```powershell
   wsl --set-default-version 2
   wsl --install -d Ubuntu-22.04
   ```
5. Launch **Ubuntu 22.04** from Start and create your UNIX username/password.
6. Confirm WSL2 is active:

   ```bash
   wsl --list --verbose
   # Should show Ubuntu-22.04 with Version 2
   ```

---

## 🔌 Step 2: VS Code Remote‑WSL

1. Open **VS Code** on Windows.
2. Press **Ctrl+Shift+X**, search for **Remote – WSL**, and **Install**.
3. Press **Ctrl+Shift+P**, type **Remote-WSL: Connect to WSL using Distro…**, select **Ubuntu-22.04**.
4. In the new VS Code window, open your home folder under WSL:

   * File → Open Folder… → `/home/<your-user>`

---

## 🤖 Step 3: Install ROS 2 Humble (Humble Hawksbill)

In your **WSL2 Ubuntu** terminal:

```bash
# 1. Update & install prerequisites
sudo apt update
sudo apt install -y curl gnupg lsb-release

# 2. Add ROS 2 apt key
echo "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg" | bash

# 3. Add the ROS 2 repo
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# 4. Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop
```

Add the ROS setup to your shell startup (optional):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🎲 Step 4: Install Gazebo & ROS–Gazebo Bridge

```bash
# 1. Enable universe repo
sudo add-apt-repository universe
sudo apt update

# 2. Install Gazebo (Jammy provides Gazebo 11.x as "gazebo")
sudo apt install -y gazebo libgazebo-dev

# 3. Install ROS 2 integration
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

Smoke-test Gazebo:

```bash
gazebo --version
# Expect: "Gazebo multi-robot simulator, version 11.x.x"
```

---

## 📂 Step 5: Clone & Build the Workspace

```bash
# 1. Clone the repo
cd ~
git clone https://github.com/apourva14/obstacle-detection-bot.git
cd obstacle-detection-bot

# 2. Install ROS dependencies
sudo rosdep init       # only if not done before
rosdep update
erosdep install --from-path src --ignore-src -y

# 3. Install colcon build tools
sudo apt install -y python3-colcon-common-extensions

# 4. Build the workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 🛠 Step 6: Fix Hard‑Coded Paths (Optional)

If any Python scripts or launch files reference absolute Linux paths (`/home/…/models/map.pgm`), update them to use package-relative paths:

```python
import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('obstacle_detection_bot')
map_file = os.path.join(pkg_share, 'models', 'map.pgm')
```

---

## ▶️ Step 7: Run the Simulation

Open two terminals in your WSL2 VS Code window:

* **Terminal A**:

  ```bash
  source install/setup.bash
  ros2 launch obstacle_detection_bot bot_world.launch.py
  ```

* **Terminal B**:

  ```bash
  source install/setup.bash
  ros2 run obstacle_detection_bot detector_node
  ```

Gazebo should pop up with the robot in the world, and the `detector_node` will process obstacles in real-time.

---

## 🐞 Troubleshooting

* **`colcon: command not found`** → `sudo apt install python3-colcon-common-extensions`
* **Gazebo won’t launch GUI** → ensure your DISPLAY is set in `~/.bashrc`:

  ```bash
  export DISPLAY=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}'):0
  export LIBGL_ALWAYS_INDIRECT=1
  ```

  And run an X server on Windows (e.g. VcXsrv).
* **Missing ROS packages** → `rosdep install --from-path src --ignore-src -y`.

---

🎉 You’re all set! Enjoy exploring obstacle detection in Gazebo with ROS 2 Humble on Windows.
