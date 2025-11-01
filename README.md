# ROS2 Virtual Robot Workspace

A development workspace for a ROS2-based virtual robot.  
Currently, the robot supports **teleoperation control**, with future updates planned to include SLAM, navigation, and autonomous mapping.

---

## 🧠 Overview

This workspace (`buddy_bot`) is being developed to simulate an autonomous robot using ROS2 and Gazebo.  
It provides a foundation for testing robot models, control systems, and mapping algorithms in a simulated environment.

**Current Capabilities:**
- Launch simulation in Gazebo
- Control robot manually using teleop
- Visualize sensors and environment in RViz

**Under Development:**
- SLAM (Simultaneous Localization and Mapping)
- Path planning & obstacle avoidance
- Sensor simulation (LIDAR, Camera)
- Full autonomous navigation

---

## 📂 Workspace Structure

```

buddy_bot/
├── src/
│   └── autonomous_bot/
│       ├── config/           # Config files (RViz, SLAM params)
│       ├── include/          # Header files
│       ├── launch/           # Launch files for simulation, mapping, SLAM
│       ├── maps/             # Maps (for SLAM / Nav2)
│       ├── src/              # Source code (nodes, scripts)
│       ├── urdf/             # Robot description and xacro files
│       ├── worlds/           # Gazebo worlds
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                    # Build directory (auto-generated)
├── install/                  # Installation directory (auto-generated)
└── log/                      # Log files (build/run output)

````

---

## ⚙️ Requirements

- **ROS2 Humble** (recommended)
- **colcon** build tool
- **Gazebo** for simulation
- **RViz2** for visualization
- **teleop_twist_keyboard** for manual control

Install teleop if missing:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
````

---

## 🚀 Build Instructions

Clone this repository and build the workspace:

```bash
# Clone the repository
git clone https://github.com/mahmudul626/buddy_bot.git
cd ~/buddy_bot

# Build the workspace
colcon build

# Source the setup file
source install/setup.bash
```

---

## ▶️ Run the Simulation

Launch the virtual robot in Gazebo:

Terminal 1
```bash
gazebo ~/buddy_bot/src/autonomous_bot/worlds/test_world.world   -slibgazebo_ros_init.so -slibgazebo_ros_factory.so
```
Terminal 2
```bash
cd buddy_bot/src/autonomous_bot/urdf/
ros2 run robot_state_publisher robot_state_publisher bot.urdf
```
Terminal 3
```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity autonomous_bot
```

Then, control it via teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

(Optional) Visualize the simulation in RViz:

```bash
rviz2
```

---

## 🧩 Future Development

Planned improvements and research directions:

* Implement SLAM using `slam_toolbox`
* Add navigation stack (`nav2`)
* Improve robot model (URDF/XACRO)
* Integrate camera and LIDAR sensors
* Autonomous exploration and mapping

---

## 🧑‍💻 Author

**Md. Mahmudul Hasan Mabud** <br>
Electronics Engineering Student | Robotics Enthusiast
[GitHub: mahmudul626](https://github.com/mahmudul626)

---

## ⚠️ Note

This workspace is **under active development**.
Expect frequent changes, unstable features, and occasional chaos as the project evolves.

---

## 🧠 License

This project is open for learning and experimentation.
Use it, modify it, and build on it freely — just don’t pretend you wrote it first.

```

---

That’s production-quality. It’s structured, version-control friendly, and communicates clearly that your project is alive and evolving — not abandoned in build logs and YAML errors.
```
