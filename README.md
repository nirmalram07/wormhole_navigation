# Multi-Map Navigation with Wormhole Transitions

## Overview

This project implements a **multi-map navigation system** using ROS 2 and TurtleBot3. The robot is capable of navigating between separate room maps using a custom "wormhole" mechanism. Each room is independently mapped and linked through doorway regions (wormholes), and transitions are managed through custom C++ nodes, a custom action server, and an integrated SQL database.

The system is designed to simulate realistic indoor navigation across multiple, spatially-disjoint environments—ideal for facilities such as warehouses, hospitals, or labs.

---

<details>
  <summary><strong>Installation Instructions (Click to expand)</strong></summary>

### Prerequisites

- ROS 2 Humble or later
- TurtleBot3 packages
- SLAM Toolbox
- SQLite3

### Installation Steps

```bash
cd ~/multi_map_nav_ws/src
git clone <your_repo_url>
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

</details>

---

## Features

- Independent mapping of rooms using SLAM Toolbox (`async_slam_toolbox_node`)
- Wormhole-based transitions between maps through overlapping doorways
- Autonomous navigation using Nav2 and AMCL
- SQL-based storage and retrieval of wormhole coordinates
- Custom C++ Action Server for inter-map goal handling

---

## Package Structure

wormhole_navigation/
├── turtlebot3_gazebo/ # Opensource package for spawning turtlebot3 in custom world
├── turtlebot3_spawn/ # Launch files for navigation and initial pose pub
├── wormhole_nav_msg/ # Custom action definition (.action file)
├── wormhole_navigation/ # Main logic: Custom action server and DB handling

```

```

```

```
