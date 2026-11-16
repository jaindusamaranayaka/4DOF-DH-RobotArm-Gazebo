# 4DOF DH Robot Arm — Gazebo Simulation

A ROS2 workspace for simulating a 4-DOF robot arm (based on the Yaskawa GP7) in Gazebo using Denavit–Hartenberg parameters.

## Project Structure

```
4DOF-DH-RobotArm-Gazebo/
├── docs/                           # Project documentation & datasheets
│   ├── Task_sheet.pdf
│   ├── EN3563_Project_Plan.pdf
│   ├── G7_manual.pdf
│   └── GP7_GP8_datasheet.pdf
├── cad/                            # CAD source files & export data
│   ├── solidworks/                 # SolidWorks source files (.SLDPRT, .SLDASM)
│   └── gp7_description.csv        # SolidWorks URDF export reference data
├── src/                            # ROS 2 packages
│   └── gp7_description/           # Main robot description package
│       ├── config/                 # Controller YAML configs
│       │   ├── controllers.yaml
│       │   └── joint_names_gp7_description.yaml
│       ├── launch/                 # ROS 2 launch files
│       │   ├── display.launch.py   # RViz visualisation
│       │   └── gazebo.launch.py    # Gazebo simulation
│       ├── meshes/                 # STL mesh files for URDF
│       ├── rviz/                   # RViz config
│       │   └── gp7.rviz
│       ├── urdf/                   # Robot description
│       │   └── gp7_description.urdf
│       ├── worlds/                 # Gazebo world files
│       │   └── gp7.world.sdf
│       ├── CMakeLists.txt
│       ├── model.config
│       └── package.xml
├── Dockerfile
└── .gitignore
```

## Folder Descriptions

| Folder | Purpose |
|--------|---------|
| `docs/` | Reference manuals, datasheets, and project planning documents |
| `cad/solidworks/` | Original SolidWorks CAD source files for the GP7 robot |
| `cad/` | SolidWorks export CSV with link/joint reference data |
| `src/gp7_description/` | ROS 2 description package (URDF, meshes, launch, config, worlds) |
| `src/gp7_description/config/` | YAML configuration for ros2_control controllers |
| `src/gp7_description/launch/` | ROS 2 Python launch files for RViz and Gazebo |
| `src/gp7_description/meshes/` | STL mesh files referenced by the URDF |
| `src/gp7_description/urdf/` | URDF robot description |
| `src/gp7_description/worlds/` | Gazebo SDF world definitions |

## Getting Started

There are **three ways** to set up this project depending on your OS. Pick the one that matches your setup.

---

### Option 1 — Docker (Recommended, works on Windows / macOS / Linux)

This is the easiest method. You only need **Docker Desktop** installed.

#### 1. Install Docker Desktop
- **Windows:** Download from [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop) and enable **WSL 2 backend** during install.
- **Linux:** Install Docker Engine via your package manager ([docs](https://docs.docker.com/engine/install/ubuntu/)).

#### 2. Clone the repository
```bash
git clone https://github.com/jaindusamaranayaka/4DOF-DH-RobotArm-Gazebo.git
cd 4DOF-DH-RobotArm-Gazebo
```

#### 3. Build the Docker image
```bash
docker build -t gp7_gazebo_env .
```
This downloads ROS 2 Humble, installs Gazebo and all dependencies, copies the `src/` folder into the container, and builds the workspace automatically.

#### 4. Run the container
```bash
docker run -it --rm gp7_gazebo_env
```
You will land inside a bash shell with ROS 2 and the workspace already sourced. To run with GUI (Gazebo window), see the **GUI Forwarding** section below.

---

### Option 2 — WSL 2 (Windows Subsystem for Linux)

Use this if you prefer working directly in a Linux terminal on Windows without Docker.

#### 1. Install WSL 2 with Ubuntu 22.04
Open **PowerShell as Administrator** and run:
```powershell
wsl --install -d Ubuntu-22.04
```
Restart your PC when prompted. After reboot, open **Ubuntu** from the Start menu and set your username/password.

#### 2. Install ROS 2 Humble
Inside your WSL Ubuntu terminal, run these commands one by one:
```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes RViz, Gazebo bridge, etc.)
sudo apt update && sudo apt install -y ros-humble-desktop
```

#### 3. Install additional dependencies
```bash
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-joint-state-publisher-gui \
    python3-colcon-common-extensions
```

#### 4. Add ROS 2 sourcing & Gazebo paths to your shell
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 5. Clone and build the workspace
```bash
git clone https://github.com/jaindusamaranayaka/4DOF-DH-RobotArm-Gazebo.git
cd 4DOF-DH-RobotArm-Gazebo
colcon build
source install/setup.bash
```

> **Tip:** If you already cloned the repo on Windows, your Windows drives are accessible inside WSL at `/mnt/c/`. You can either work there or clone fresh into the WSL home directory (`~/`) for better file-system performance.

---

### Option 3 — Native Ubuntu 22.04 (Dual-boot or VM)

#### 1. Install ROS 2 Humble
Follow the same steps as **Option 2, Steps 2–4** above (they are identical on native Ubuntu 22.04).

#### 2. Clone and build
```bash
git clone https://github.com/jaindusamaranayaka/4DOF-DH-RobotArm-Gazebo.git
cd 4DOF-DH-RobotArm-Gazebo
colcon build
source install/setup.bash
```

---

## Running the Simulation & Pick-and-Place Mission

After building (via any option above):

### 1. Run Complete Autonomous Pick-and-Place
Launches Gazebo, spawns the GP7 arm and boxes, starts controllers, and executes the pick-and-place mission automatically:
```bash
ros2 launch gp7_description pick_and_place.launch.py
```

### 2. Or Run Separately (Manual)
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch gp7_description gazebo.launch.py

# Terminal 2: Run pick-and-place Python controller
ros2 run gp7_description pick_and_place.py
```

### 3. View the Robot in RViz (Model Inspection)
```bash
ros2 launch gp7_description display.launch.py
```

---

## GUI Forwarding (Docker & WSL)

To see the Gazebo / RViz window from Docker or WSL you need an X server on Windows:

1. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (or use the built-in WSLg on Windows 11).
2. Launch VcXsrv with **"Disable access control"** checked.
3. In your WSL or Docker terminal, set the display:
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   export LIBGL_ALWAYS_INDIRECT=0
   ```
   > On **Windows 11 with WSLg**, GUI apps work out of the box — no extra setup needed.

For Docker, run the container with display forwarding:
```bash
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    gp7_gazebo_env
```

---

## Syncing Between Windows and Linux

If you edit files on Windows (e.g., SolidWorks URDF export) and need them on your Linux/WSL side:

```bash
# On your Linux / WSL terminal
cd ~/4DOF-DH-RobotArm-Gazebo
git pull
colcon build
source install/setup.bash
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `colcon build` fails with missing packages | Run the `apt install` commands from Step 3 of Option 2 |
| Gazebo window doesn't open (WSL) | Make sure VcXsrv is running, or upgrade to Windows 11 for WSLg |
| `docker build` fails at network step | Check your internet connection; Docker needs to download ~2 GB |
| Permission denied on `git push` | Set up SSH keys: `ssh-keygen -t ed25519` then add the key to GitHub |
| `Failed to load system plugin [gz_ros2_control-system]` | On ROS 2 Humble (Ignition Fortress), ensure `ros-humble-ign-ros2-control` is installed and the URDF references `ign_ros2_control-system` |
| `Unable to find file with URI [model://gp7_description/...]` | Ensure `IGN_GAZEBO_RESOURCE_PATH` (or `GZ_SIM_RESOURCE_PATH`) contains your workspace `install/share` directory |
| Controller spawners timing out on `/controller_manager/list_controllers` | This happens when Gazebo's ros2_control plugin fails to load. Fixing the plugin name/package resolves this |

---

## Team
EN3563 — Robotics Group Project
