# Data Transmission Logic for TurtleBot3 Ball Collection Project

## Overview

I will show you the data transmission logic and specific locations for the entire TurtleBot3 ball collection project. It details how data flows between nodes, the topics used, message types, and the specific files where each component is implemented.

## System Architecture

The project consists of the following nodes:

1. **yolo\_detector\_node** - Detects balls using YOLO and publishes dynamic 3D poses array.
2. **density\_map\_builder\_node** - Builds a dynamic Gaussian density map from ball poses, extracts peak targets, and directly interfaces with the Nav2 stack.

## Detailed Data Transmission Logic

### 1. Ball Detection

**Node**: `yolo_detector_node`
**File**: `turtlebot3_vision` 

**Data Inputs**:

- **Topic**: `/camera/color/image_raw` type: `sensor_msgs/Image`
- Topic: `/camera/depth/image_rect_raw `   type: `sensor_msgs/Image`

**Data Outputs**:

- **Topic**: `/vision/target_poses` type: `geometry_msgs/PoseArray`
  - Purpose: Publishes detected ball 3D poses from YOLO + depth.

### 2. Density Map Construction

**Node**: `density_map_builder_node`
**File**: `src/density_map_builder_node.cpp`

**Data Inputs**:

- **Topic**: `/vision/target_poses` type:`geometry_msgs/PoseArray`
  - Purpose:Receives dynamic 3D poses from the vision node.

**Data Outputs**:

- **Topic**: `/visualization/density_markers` type:`visualization_msgs/MarkerArray`
  - Purpose:Publishes height-mapped colored grids in RViz to visualize the 2D Gaussian hills.

- **Action Client**: `/navigate_to_pose` type:`nav2_msgs/action/NavigateToPose`
Purpose: Directly sends the coordinate $(X_{max}, Y_{max})$ of the highest potential field peak to the Nav2 stack for immediate execution.


## Launch Configuration

**File**: `launch/ball_collection.launch.py`

**Launch Arguments**:

- `params_file`: Path to parameter file (default: `param/ball_collection.yaml`)
- `ball_count`: Number of balls to spawn (default: 20)

**Launched Nodes**:

- `yolo_detector_node`: Subscribes to RGB-D camera feeds, runs YOLO detection, and publishes the target 3D poses.
- `density_map_builder_node`: Generates a dynamic spatial density map and directly sends goal points to the Nav2 action server.
- `rotate_once`: Automatically rotates the TurtleBot3 360 degrees upon launch to securely initialize map coordinate targets.
- `ekf_filter_node` (via `full_system.launch`): Fuses wheel odometry with IMU data for smoother pose tracking.

## Parameter Files

- `param/ekf.yaml`: Configures the `robot_localization` parameters. Defines trusted vectors for `/odom` and `/imu`, mapping frames, and disables standard `publish_tf` to avoid competing with Gazebo's native local transforms.
- `param/slam_toolbox.yaml`: Adjusts map correlation layers, loop closure distances, and ROS frame constraints for asynchronous 2D mapping.

## Visualization
**RViz Topics**:

- `/visualization/density_markers` (`visualization_msgs/MarkerArray`): Graphically renders the spatial density model (using colorized 3D blocks to depict coordinate "value").
- `/map` (`nav_msgs/OccupancyGrid`): Visualizes real-time walls and objects mapped by the SLAM toolbox.
- `/camera/color/image_raw` (`sensor_msgs/Image`): Display camera input frame.
- `/scan` (`sensor_msgs/LaserScan`): The primary environmental readings from the LiDAR.

## Conclusion

The data transmission logic for the TurtleBot3 ball collection project follows a clear flow from ball detection to path planning. Each node has specific responsibilities and communicates through well-defined topics. The system is designed to:

1. Detect balls using YOLO directly from camera RGB-D streams
2. Construct a dynamic density map with Gaussian distribution and time decay
3. Identify high-density areas (peak Gaussian spots)
4. Dispatch navigation goals directly to the highest-density areas via Nav2 stack action server

This architecture ensures efficient ball collection by prioritizing high-density areas, significantly enhancing recovery efficiency within a single battery life.
