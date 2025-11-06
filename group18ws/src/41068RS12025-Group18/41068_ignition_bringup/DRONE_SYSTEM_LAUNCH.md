# Drone Complete System Launch

This document describes how to use the complete drone system launch file that starts all nodes automatically.

## Overview

The `drone_complete_system.launch.py` launch file starts all necessary nodes for the drone system in a single command. It replaces the need to manually start 5 separate terminals.

## Quick Start

```bash
cd ~/41068_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/41068_ws/install/setup.bash

ros2 launch 41068_ignition_bringup drone_complete_system.launch.py
```

## What Gets Launched

The launch file starts the following components in order:

1. **Base Simulation** (immediate start)
   - Gazebo simulation
   - Robot state publisher
   - Robot localization (EKF)
   - Gazebo bridge
   - RViz (if enabled)
   - Nav2 navigation stack (if enabled)

2. **Path Planning** (starts after 5-6 seconds)
   - `planner_node` - Computes paths using Nav2
   - `path_planner_service_node` - Provides path retrieval service

3. **Drone Controllers** (starts after 7-8 seconds)
   - `altitude_controller` - Controls drone altitude
   - `mission_node` - Executes missions with planned paths

4. **Sensors** (starts after 9 seconds)
   - `lidar_tree_detector_node` - Detects trees using LiDAR

5. **User Interface** (starts after 10 seconds)
   - `drone_ui_node` - Web-based control interface

## Launch Arguments

### Default Values
- `world`: `Plantation2`
- `slam`: `True`
- `nav2`: `True`
- `rviz`: `True`
- `use_sim_time`: `True`
- `color_detection`: `False`

### Examples

**Basic launch with defaults:**
```bash
ros2 launch 41068_ignition_bringup drone_complete_system.launch.py
```

**Launch with different world:**
```bash
ros2 launch 41068_ignition_bringup drone_complete_system.launch.py world:=large_demo
```

**Launch without RViz (for headless operation):**
```bash
ros2 launch 41068_ignition_bringup drone_complete_system.launch.py rviz:=False
```

**Launch with camera-based tree color detection:**
```bash
ros2 launch 41068_ignition_bringup drone_complete_system.launch.py color_detection:=True
```

## Startup Timing

The nodes are started with delays to ensure proper initialization:

- **0-5s**: Base simulation and navigation stack initialize
- **5s**: Path planner starts
- **6s**: Path planner service starts (after planner_node)
- **7s**: Altitude controller starts
- **8s**: Mission node starts
- **9s**: LiDAR tree detector starts
- **10s**: UI starts (everything is ready)

## Usage Workflow

1. **Launch the system:**
   ```bash
   ros2 launch 41068_ignition_bringup drone_complete_system.launch.py
   ```

2. **Wait for UI to appear** (approximately 10-15 seconds)
   - The UI will open in a window showing drone controls

3. **Plan a path:**
   - Enter X, Y, Z coordinates in the UI
   - Click "Plan Path to Goal"
   - Wait for confirmation that path is planned

4. **Start the mission:**
   - Click "Start Drone" button
   - The drone will:
     - Take off to the configured altitude
     - Retrieve the planned path from the path planner service
     - Follow the waypoints to reach the goal

5. **Monitor progress:**
   - Watch the drone status in the UI
   - Observe the drone in RViz and Gazebo
   - Use Stop/Return Home buttons as needed

## Available Worlds

- `simple_trees` - Basic world with a few trees
- `large_demo` - Larger world with more trees
- `PlantationTest` - Plantation test environment
- `Plantation2` - Main plantation world (default)

## Troubleshooting

### Nodes not starting
If nodes fail to start, check the terminal output for errors. Common issues:
- Navigation stack not ready (wait longer before starting)
- Path planner service not available (ensure planner_node is running)

### Path planner service unavailable
If the mission node reports "Path planner service not available":
- Check that `planner_node` is running: `ros2 node list | grep planner`
- Check that `path_planner_service_node` is running
- Ensure you've planned a path in the UI before clicking "Start Drone"

### Drone doesn't follow planned path
If the drone follows hardcoded waypoints instead:
- Ensure you've planned a path in the UI **before** clicking "Start Drone"
- Check the mission_node output for "Received planned path with X waypoints"
- If you see "Falling back to hardcoded waypoints", the path service isn't responding

### UI doesn't appear
The UI takes about 10-15 seconds to start. If it still doesn't appear:
- Check that all nodes have started: `ros2 node list`
- Look for errors in the terminal output
- Ensure `drone_ui` package is built correctly

## Comparison with Manual Launch

### Before (5 terminals):
```bash
# Terminal 1
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=Plantation2

# Terminal 2
ros2 run drone_controller mission_node

# Terminal 3
ros2 run drone_controller altitude_controller

# Terminal 4
ros2 run drone_ui drone_ui_node

# Terminal 5
ros2 run lidar_tree_detector lidar_tree_detector_node
```

### After (1 command):
```bash
ros2 launch 41068_ignition_bringup drone_complete_system.launch.py
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│              Gazebo Simulation                      │
│  (World, Physics, Robot, Sensors)                   │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────┴────────────────────────────────────┐
│              Nav2 Stack                             │
│  (SLAM, Localization, Path Planning)                │
└────────────────┬────────────────────────────────────┘
                 │
         ┌───────┴───────┐
         │               │
    ┌────▼─────┐   ┌────▼──────────┐
    │ Planner  │   │  Altitude     │
    │ Node     │   │  Controller   │
    └────┬─────┘   └───────────────┘
         │
    ┌────▼──────────────┐
    │ Path Planner      │
    │ Service Node      │
    └────┬──────────────┘
         │
    ┌────▼─────┐   ┌─────────────┐   ┌──────────┐
    │ Mission  │   │  LiDAR Tree │   │ Drone UI │
    │ Node     │◄──┤  Detector   │◄──┤  Node    │
    └──────────┘   └─────────────┘   └──────────┘
```

## Additional Resources

- Base simulation README: `41068_ignition_bringup/README.md`
- Path planner README: `path_planner_cpp/README.md`
- Drone controller source: `drone_controller/src/`

## Notes

- **IMPORTANT**: Always plan a path in the UI before clicking "Start Drone"
- The mission node will fall back to hardcoded waypoints if no path is planned
- You can stop and restart missions using the UI controls
- Press Ctrl+C in the terminal to stop all nodes
