# TurtleBot3 Ball Collection Package

This package implements a complete ball collection pipeline for TurtleBot3 robots, including vision-based detection, clustering, path planning, and navigation.

## Features

- **Vision Simulation**: Reads ball positions from spawn script and publishes as point cloud
- **DBSCAN Clustering**: Groups scattered balls into high-value centroids
- **TSP Planning**: Optimizes collection order using greedy TSP algorithm
- **Navigation Integration**: Uses Nav2 for path planning and execution
- **SLAM Support**: Optional SLAM Toolbox integration for online mapping

## Launch Options

### Full System with Pre-built Map
```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py use_slam:=false
```

### Full System with SLAM
```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py use_slam:=true
```

### SLAM + Navigation Only
```bash
ros2 launch turtlebot3_ball_collection slam_navigation.launch.py
```

## Parameters

- `eps`: DBSCAN epsilon parameter (default: 1.0)
- `min_samples`: DBSCAN minimum samples (default: 2)
- SLAM parameters in `param/slam_toolbox.yaml`

## Topics

- `/ball_positions`: Detected ball positions (PointCloud2)
- `/centroids`: Clustered centroids (PointCloud2)
- `/collection_path`: Optimized collection path (Path)
- `/ball_markers`, `/centroid_markers`: Visualization markers

## Dependencies

- ROS2 Humble
- Nav2
- PCL
- SLAM Toolbox (optional)

## Usage

1. Set environment variables:
   ```bash
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-01
   ```

2. Launch the system:
   ```bash
   ros2 launch turtlebot3_ball_collection full_system.launch.py
   ```

3. The robot will automatically detect balls, plan a collection path, and navigate to the first target.

## SLAM Integration

When `use_slam:=true`, the system uses SLAM Toolbox for online mapping instead of a pre-built map. This is useful for:

- Unknown environments
- Dynamic obstacle avoidance
- Real-time map updates

The SLAM node publishes to `/map` topic, which Nav2 uses for navigation.