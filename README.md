# 4DOF DH Robot Arm — Gazebo Simulation

A ROS2 workspace for simulating a 4-DOF robot arm (based on the Yaskawa GP7) in Gazebo using Denavit–Hartenberg parameters.

## Project Structure

```
4DOF-DH-RobotArm-Gazebo/
├── docs/                  # Project documentation & datasheets
│   ├── Task_sheet.pdf
│   ├── EN3563_Project_Plan.pdf
│   ├── G7_manual.pdf
│   └── GP7_GP8_datasheet.pdf
├── cad/                   # CAD files & robot models
│   ├── solidworks/        # SolidWorks source files (.SLDPRT, .SLDASM)
│   │   └── GP7 components/
│   ├── meshes/            # Exported mesh files for URDF/Gazebo (.stl, .dae)
│   └── urdf/              # URDF / XACRO robot description files
├── src/                   # ROS2 packages
├── config/                # ROS2 config & parameter YAML files
├── launch/                # ROS2 launch files
├── worlds/                # Gazebo world files (.sdf, .world)
└── .gitignore
```

## Folder Descriptions

| Folder | Purpose |
|--------|---------|
| `docs/` | Reference manuals, datasheets, and project planning documents |
| `cad/solidworks/` | Original SolidWorks CAD source files for the GP7 robot |
| `cad/meshes/` | Exported STL/DAE mesh files for use in URDF and Gazebo |
| `cad/urdf/` | URDF and XACRO files describing robot kinematics & visuals |
| `src/` | ROS2 packages (controllers, kinematics, planning, etc.) |
| `config/` | YAML configuration files for ROS2 nodes and controllers |
| `launch/` | ROS2 launch files to start simulation and nodes |
| `worlds/` | Gazebo world definitions for simulation environments |

## Getting Started

### Prerequisites
- ROS2 (Humble or later)
- Gazebo (Fortress or later)
- `colcon` build tool

### Build
```bash
cd 4DOF-DH-RobotArm-Gazebo
colcon build
source install/setup.bash
```

## Team
EN3563 — Robotics Group Project
