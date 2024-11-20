# dmap_live_registration

The `dmap_live_registration` package is a ROS Noetic-based package designed to perform dynamic mapping using laser scan data. The package generates a grid map in real-time, visualizes it on a canvas, and publishes it as an occupancy grid for further processing or visualization in RViz.

## Overview

This package integrates multiple ROS nodes to:
1. Simulate a robot environment using `stage_ros`.
2. Publish laser scan data and ground truth poses.
3. Dynamically map the environment using a custom live registration node.
4. Visualize the environment in RViz and control the robot via keyboard teleoperation.

## Package Contents

### Launch File

The main launch file initializes the following nodes with their corresponding parameters:

- **`stageros`**: Simulates the robot environment using the specified world file.
- **`map_server`**: Loads a preconfigured map.
- **`static_transform_publisher`**: Publishes a static transformation between `odom` and `map`.
- **`dmap_live_registration`**: Performs live mapping using laser scan data and publishes a grid map.
- **`rviz`**: Visualizes the environment, including the robot and grid map.
- **`teleop_twist_keyboard`**: Enables manual control of the robot via keyboard input.

### Nodes and Topics

#### `stageros` Node
- **Package**: `stage_ros`
- **Type**: `stageros`
- **Purpose**: Simulates the robot environment and generates topics such as `/base_scan` and `/base_pose_ground_truth`.
- **Parameters**:
  - `world_file`: Path to the Stage world file.
- **Topics**:
  - `/base_scan`: Publishes laser scan data (sensor_msgs/LaserScan).
  - `/base_pose_ground_truth`: Publishes the ground truth pose of the robot.

#### `map_server` Node
- **Package**: `map_server`
- **Type**: `map_server`
- **Purpose**: Loads and serves a static map.
- **Parameters**:
  - `map_param`: Path to the map YAML file.
- **Topics**:
  - `/map`: The loaded map (nav_msgs/OccupancyGrid).
  - `/map_metadata`: Metadata for the map.

#### `static_transform_publisher` Node
- **Package**: `tf`
- **Type**: `static_transform_publisher`
- **Purpose**: Publishes a static transform between `odom` and `map`.
- **Parameters**:
  - Transformation parameters: Translation (-4, -13, 0) and rotation (0, 0, 0, 1).
- **Topics**:
  - `/tf_static`: Static transformations.

#### `dmap_live_registration` Node
- **Package**: `dmap_live_registration`
- **Type**: `dmap_localize_node`
- **Purpose**: Processes laser scan data to dynamically generate a grid map and publishes it as an occupancy grid.
- **Subscribed Topics**:
  - `/base_scan`: Laser scan data (sensor_msgs/LaserScan).
- **Published Topics**:
  - `/gridmap`: Generated grid map (nav_msgs/OccupancyGrid).

**Detailed Functionality**:
- **Inputs**: Laser scan data from `/base_scan`.
- **Processing**:
  1. Converts the laser scan points into grid coordinates using the `GridMapping` utility.
  2. Performs dynamic mapping using `DMap` to compute distances and grid data.
  3. Generates a `Grid_<float>` representing the environment and converts it to `nav_msgs/OccupancyGrid`.
- **Outputs**: Publishes the grid map to `/gridmap`.

#### `rviz` Node
- **Package**: `rviz`
- **Type**: `rviz`
- **Purpose**: Visualizes the robot, laser scan data, and grid map in a 3D view.
- **Parameters**:
  - Path to the RViz configuration file (`rviz2.rviz`).

#### `teleop` Node
- **Package**: `teleop_twist_keyboard`
- **Type**: `teleop_twist_keyboard.py`
- **Purpose**: Allows manual control of the robot via keyboard inputs.
- **Topics**:
  - `/cmd_vel`: Publishes velocity commands to control the robot.

### Topics Overview

| **Topic**                  | **Message Type**             | **Description**                                         |
|----------------------------|------------------------------|---------------------------------------------------------|
| `/base_pose_ground_truth`  | `geometry_msgs/Pose`         | Ground truth pose of the robot.                        |
| `/base_scan`               | `sensor_msgs/LaserScan`      | Laser scan data generated by `stageros`.               |
| `/gridmap`                 | `nav_msgs/OccupancyGrid`     | Dynamically generated grid map.                        |
| `/map`                     | `nav_msgs/OccupancyGrid`     | Static map loaded by `map_server`.                     |
| `/map_metadata`            | `nav_msgs/MapMetaData`       | Metadata about the static map.                         |
| `/cmd_vel`                 | `geometry_msgs/Twist`        | Velocity commands for robot control.                   |
| `/tf`                      | `tf2_msgs/TFMessage`         | Dynamic transform information.                         |
| `/tf_static`               | `tf2_msgs/TFMessage`         | Static transform information between `odom` and `map`. |

### Parameters

| **Parameter**              | **Default Value**                                    | **Description**                          |
|----------------------------|-----------------------------------------------------|------------------------------------------|
| `node`                     | `dmap_localize_node`                                | Node type for live mapping.              |
| `world_file`               | `config/cappero_laser_odom_diag_obstacle_2020...`   | Path to the Stage world file.            |
| `map_param`                | `config/map.yaml`                                   | Path to the static map YAML file.        |

## How to Use

1. Clone the repository and build the workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone <repository_url>
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
