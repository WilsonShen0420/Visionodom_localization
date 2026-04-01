# CLAUDE.md

## Project Overview

Visual odometry based redundant localization system for ROS Noetic mobile robots.
Runs alongside LiDAR SLAM to provide camera-based pose estimation using RTAB-Map,
and can automatically correct drifted SLAM via initialpose service calls.

## Repository Structure

```
visual_localization/          # ROS Noetic catkin package
├── src/
│   └── visual_localization_node.cpp   # Main C++ node
├── srv/
│   ├── SetAbsolutePose.srv            # Service: receive absolute pose from external
│   └── SetInitialPose.srv             # Service: SLAM correction (ROS1 equivalent of nav2_msgs)
├── config/
│   └── params.yaml                    # All tunable parameters
├── launch/
│   ├── visual_localization.launch     # Full system (RTAB-Map VO + localization node)
│   └── rtabmap_vo.launch              # RTAB-Map visual odometry only
├── CMakeLists.txt
└── package.xml
```

## Build

```bash
# In catkin workspace
cd ~/catkin_ws/src
ln -s /path/to/Visionodom_localization/visual_localization .
cd ~/catkin_ws
catkin_make --pkg visual_localization
# or
catkin build visual_localization
```

## Run

```bash
# RGB-D camera (RealSense)
roslaunch visual_localization visual_localization.launch camera_type:=rgbd

# Stereo camera (LeTMC-520)
roslaunch visual_localization visual_localization.launch camera_type:=stereo

# With IMU fusion
roslaunch visual_localization visual_localization.launch camera_type:=rgbd use_imu:=true
```

## Architecture

### Node: `visual_localization_node`

Two operating modes:
- **RELATIVE_ONLY** (startup default): publishes `vo_odom -> camera_link` TF from VO data
- **ABSOLUTE_MODE** (after `~/set_absolute_pose` service call): additionally publishes `map -> vo_odom` TF, enabling `map -> camera_link` lookup; monitors LiDAR SLAM and triggers correction when drift exceeds thresholds

### ROS Interfaces

| Direction | Name | Type | Purpose |
|-----------|------|------|---------|
| Sub | `/vo/odom` | `nav_msgs/Odometry` | RTAB-Map VO output |
| Sub | `/slam_pose` | `geometry_msgs/PoseStamped` | LiDAR SLAM pose |
| Pub | `~/pose` | `geometry_msgs/PoseStamped` | Absolute pose (in ABSOLUTE_MODE) |
| TF | `vo_odom -> camera_link` | - | Always published |
| TF | `map -> vo_odom` | - | Published in ABSOLUTE_MODE |
| Srv Server | `~/set_absolute_pose` | `SetAbsolutePose` | Inject absolute pose reference |
| Srv Client | `/set_pose` | `SetInitialPose` | Correct drifted LiDAR SLAM |

### Coordinate Math

```
T_map_to_odom = T_absolute_anchor * inv(T_vo_at_anchor)
T_absolute(t) = T_map_to_odom * T_vo(t)
```

### SLAM Correction Safeguards

- Consecutive threshold: must exceed N consecutive checks (default 5)
- Cooldown: minimum 30s between corrections
- Dual threshold: translation (0.5m) and rotation (0.3rad) independently checked

## Key Dependencies

- `rtabmap_ros` (visual odometry engine)
- `tf2`, `tf2_ros`, `tf2_eigen`, `eigen_conversions`
- `geometry_msgs`, `nav_msgs`

## Hardware Support

- **RGB-D**: RealSense D435, D455, etc.
- **Stereo**: LeTMC-520
- **IMU**: Optional, improves VO stability when enabled

## Development Notes

- Language: C++17
- Thread safety: dual mutex design (`mutex_` for VO/mode state, `slam_mutex_` for SLAM pose)
- RTAB-Map `publish_tf` is disabled; this node manages all TF broadcasting
- `SetInitialPose.srv` is a ROS1 equivalent since `nav2_msgs` only exists in ROS2; adapt if your SLAM system uses a different service type
- All topic names, frame IDs, and thresholds are configurable via `config/params.yaml` or launch args
