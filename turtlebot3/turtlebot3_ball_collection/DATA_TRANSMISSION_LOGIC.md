# Data Transmission Logic for TurtleBot3 Ball Collection

## Overview
This document describes the current 2-node architecture and the refined planning logic:

- Keep the same two nodes.
- Add an A* cost model in `density_map_builder_node`.
- Make high-density (peak-related) cells cheaper while still considering travel distance.

## System Architecture
The project consists of two nodes:

1. `yolo_detector_node`
- Detects balls from RGB-D input.
- Publishes dynamic 3D target poses.

2. `density_map_builder_node`
- Receives target poses.
- Builds a dynamic Gaussian density map with time decay.
- Extracts peak target(s).
- Runs A* with a hybrid cost: distance + density reward.
- Sends the selected goal to Nav2.

## Data Flow
1. Camera input to `yolo_detector_node`
- `/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`)

2. Vision output
- `/vision/target_poses` (`geometry_msgs/PoseArray`)

3. Density and planning in `density_map_builder_node`
- Subscribes `/vision/target_poses`
- Publishes `/visualization/density_markers` (`visualization_msgs/MarkerArray`)
- Sends navigation goal through `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## Refined A* Planning Logic
### Goal
Use peak information from the Gaussian map as a semantic reward so that:

- Higher density means lower traversal cost.
- Distance cost is still active.

### Notation
- `D(i)`: density value of cell `i`
- `D_norm(i)`: normalized density in `[0, 1]`
- `d_step(i, j)`: geometric step distance from cell `i` to `j`
- `w_dist`: distance weight
- `w_peak`: peak reward weight

### Density normalization
Use min-max normalization per update cycle:

`D_norm(i) = clamp((D(i) - D_min) / (D_max - D_min + eps), 0, 1)`

### Traversal cost per expansion
For neighbor expansion `i -> j` in A*:

`c(i, j) = w_dist * d_step(i, j) + w_peak * (1 - D_norm(j))`

Interpretation:
- If `D_norm(j)` is high (near peak), `(1 - D_norm(j))` is small, so cell cost decreases.
- If `D_norm(j)` is low, the penalty increases.

### A* objective
Standard form:

- `g(j) = g(i) + c(i, j)`
- `f(j) = g(j) + h(j)`

Use Euclidean heuristic to the chosen peak (or selected target):

`h(j) = w_h * ||j - goal||_2`

### Peak selection policy
Before A*:
- Extract one or more local maxima from density map.
- Option A: choose global peak as single goal.
- Option B: choose top-k peaks, run A* for each, select minimal total objective.

## Practical Parameter Guidance
- Start with `w_dist = 1.0`, `w_peak = 0.4 ~ 1.2`.
- If robot ignores dense regions: increase `w_peak`.
- If robot detours too much: decrease `w_peak`.
- Keep a minimum peak threshold to avoid chasing noise.

## Launch and Runtime Notes
- File: `launch/ball_collection.launch.py`
- Core nodes:
  - `yolo_detector_node`
  - `density_map_builder_node`
  - optional `rotate_once.py` for initial scan
- SLAM/Nav2 are launched from `full_system.launch.py`

## Conclusion
planning is upgraded from "directly go to the max peak point" to "A* with semantic density reward + distance cost". This keeps path feasibility while biasing the robot toward high-yield ball regions.
