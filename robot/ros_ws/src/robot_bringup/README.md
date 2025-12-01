# Robot Bringup - Wire Perching Configuration

## Overview

This package provides launch files for bringing up the complete robot system, including the autonomy stack configured for wire perching missions.

## Launch Files

### Main Launch Files

#### `robot.launch.xml`
Standard robot bringup with default autonomy configuration.

**Environment Variables Required:**
- `ROBOT_NAME`: Robot namespace (e.g., `robot_1`)
- `ROBOT_URDF_FILE_PATH`: Path to robot URDF file
- `AUTONOMY_LAUNCH_PACKAGE`: Package containing autonomy launch file
- `AUTONOMY_LAUNCH_FILE`: Autonomy launch file to use

**Launch Arguments:**
- `sim` (default: `true`): Whether running in simulation or on real hardware

**Usage:**
```bash
ros2 launch robot_bringup robot.launch.xml
```

#### `robot_wire_perching.launch.xml`
Robot bringup configured specifically for wire perching missions. This launch file:
- Launches the complete autonomy stack with wire perception enabled
- Includes wire detection and tracking nodes
- Enables wire approach planner
- Configures behavior executive for perch/detach sequences

**Usage:**
```bash
ros2 launch robot_bringup robot_wire_perching.launch.xml
```

### Simulation Launch Files

#### `sim.launch.xml`
Launches stereo processing nodes for simulation environments:
- Stereo disparity computation
- Point cloud generation
- RViz visualization
- Behavior GUI

### Real Hardware Launch Files

#### `real.launch.xml`
Configuration for running on real hardware:
- ZED camera static transforms
- MAVROS interface to flight controller
- Optional bag recording

## Wire Perching Stack Components

The wire perching configuration (`robot_wire_perching.launch.xml`) launches:

### 1. Interface Layer (0_interface)
- **MAVROS**: Communication with PX4/ArduPilot flight controller
- **Robot Interface**: High-level robot abstraction
- **Safety Monitor**: Emergency handling and safety checks

### 2. Sensors Layer (1_sensors)
- **Gimbal Stabilizer**: Camera gimbal control
- **Camera Parameters**: Camera calibration server

### 3. Perception Layer (2_perception)
- **MACVO**: Visual-inertial odometry
- **Wire Detection**: Hough + RANSAC-based wire detection from RGB-D
- **Wire Tracking**: Kalman filter-based temporal tracking

### 4. Local Layer (3_local)
- **Trajectory Controller**: Trajectory tracking control
- **PID Controller**: Low-level position/velocity control
- **Takeoff/Landing Planner**: Specialized takeoff/landing trajectories
- **Wire Approach Planner**: Generates approach trajectories to detected wires
- **DROAN Local Planner**: Obstacle avoidance planner
- **Disparity Expansion**: Local obstacle map from stereo

### 5. Global Layer (4_global)
- **VDB Mapping**: Global 3D mapping
- **Random Walk Planner**: Exploration planning

### 6. Behavior Layer (5_behavior)
- **Behavior Tree**: Mission execution framework
- **Behavior Executive**: High-level decision making with perch/detach actions

## Topic Namespacing

All topics are namespaced under `$(env ROBOT_NAME)` (e.g., `/robot_1/`):

### Wire Perception Topics
- `/robot_1/perception/wire_perception/wire_detections` - Raw wire detections
- `/robot_1/perception/wire_perception/wire_target` - Filtered wire target
- `/robot_1/perception/wire_perception/wire_markers` - Visualization markers

### Wire Approach Topics
- `/robot_1/wire_approach_planner/wire_aligned` - Alignment status (Bool)
- `/robot_1/wire_approach_planner/enable` - Enable/disable service

### Control Topics
- `/robot_1/trajectory_controller/trajectory_segment_to_add` - Trajectory commands
- `/robot_1/trajectory_controller/tracking_point` - Current tracking point
- `/robot_1/interface/cmd_roll_pitch_yawrate_thrust` - Low-level control

## Environment Setup

### Using Docker (Recommended)

The `.env` file at the repository root configures the autonomy stack:

```bash
# For wire perching missions
ROBOT_LAUNCH_PACKAGE="robot_bringup"
ROBOT_LAUNCH_FILE="robot_wire_perching.launch.xml"

# Component launch files (used by autonomy_wire_perching.launch.xml)
INTERFACE_LAUNCH_PACKAGE="interface_bringup"
INTERFACE_LAUNCH_FILE="interface.launch.xml"

SENSORS_LAUNCH_PACKAGE="sensors_bringup"
SENSORS_LAUNCH_FILE="sensors.launch.xml"

# Perception uses wire detection/tracking
# (hardcoded in autonomy_wire_perching.launch.xml)

# Local uses wire approach planner
# (hardcoded in local_wire.launch.xml)

GLOBAL_LAUNCH_PACKAGE="global_bringup"
GLOBAL_LAUNCH_FILE="global.launch.xml"

BEHAVIOR_LAUNCH_PACKAGE="behavior_bringup"
BEHAVIOR_LAUNCH_FILE="behavior.launch.xml"

# Robot description
ROBOT_DESCRIPTION_PACKAGE="iris_with_sensors_description"
ROBOT_URDF_FILE="iris_with_sensors.pegasus.robot.urdf"
```

### Launch via Docker Compose

```bash
# From repository root
docker compose up robot

# Or with explicit profile
docker compose --profile sitl up robot
```

### Manual Launch (Without Docker)

```bash
# Set required environment variables
export ROBOT_NAME="robot_1"
export ROBOT_URDF_FILE_PATH="/path/to/urdf"

# Source workspace
source /home/robot/AirStack/robot/ros_ws/install/setup.bash

# Launch
ros2 launch robot_bringup robot_wire_perching.launch.xml
```

## Behavior Tree Commands

The wire perching mission uses these behavior tree conditions/actions:

### Perching Sequence
1. **Condition**: `Perch Mission Commanded` - User initiates perch
2. **Action**: `Approach Wire` - Track and approach wire target
3. **Condition**: `Wire Aligned` - Wire approach planner reports alignment
4. **Action**: `Hover Perch Position` - Stabilize before clamping
5. **Action**: `Clamp Wire` - Actuate gripper to grasp wire
6. **Condition**: `Perch Complete` - Successfully perched

### Detaching Sequence
1. **Condition**: `Detach Mission Commanded` - User initiates detach
2. **Action**: `Hover Detached Position` - Stabilize before release
3. **Condition**: `Detach Hovering` - Hover stable
4. **Action**: `Unclamp Wire` - Release gripper
5. **Condition**: `Wire Unclamped` - Gripper released
6. **Condition**: `Detach Completed` - Successfully detached

## Configuration Files

### Wire Detection Config
`robot/ros_ws/src/autonomy/2_perception/wire_perception/config/wire_detection_config.yaml`

### Wire Tracking Config
`robot/ros_ws/src/autonomy/2_perception/wire_perception/config/wire_tracking_config.yaml`

### Wire Approach Planner Config
`robot/ros_ws/src/autonomy/3_local/b_planners/wire_approach_planner/config/wire_approach_config.yaml`

### Behavior Tree Config
`robot/ros_ws/src/autonomy/5_behavior/behavior_tree/config/drone.tree`

## Troubleshooting

### Wire Not Detected
- Check RGB and depth topics are publishing: `ros2 topic hz /robot_1/sensors/front_stereo/left/image_rect`
- Verify camera_info is available: `ros2 topic echo /robot_1/sensors/front_stereo/left/camera_info`
- Monitor detection topic: `ros2 topic echo /robot_1/perception/wire_perception/wire_detections`
- Adjust Hough/RANSAC parameters in `wire_detection_config.yaml`

### Wire Tracking Issues
- Check odometry is publishing: `ros2 topic hz /robot_1/odometry_conversion/odometry`
- Verify Kalman filter convergence in logs
- Adjust process/measurement noise in `wire_tracking_config.yaml`

### Wire Approach Not Working
- Ensure wire target is being published: `ros2 topic echo /robot_1/perception/wire_perception/wire_target`
- Check if planner is enabled: `ros2 service call /robot_1/wire_approach_planner/enable std_srvs/srv/SetBool "{data: true}"`
- Monitor trajectory generation: `ros2 topic echo /robot_1/trajectory_controller/trajectory_segment_to_add`

### Build Issues
```bash
# Clean build
cd ~/AirStack/robot/ros_ws
rm -rf build/ install/ log/

# Build with verbose output
colcon build --packages-select wire_perception hummingbird_msgs --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Development

### Adding New Launch Configurations

1. Create new launch file in `launch/` directory
2. Update `CMakeLists.txt` to install it
3. Document in this README
4. Test both in simulation and on hardware

### Modifying Wire Perception Pipeline

The wire perception pipeline can be customized by:
- Editing `perception_wire.launch.xml` to change parameters
- Creating variant launch files (e.g., `perception_wire_gpu.launch.xml` for GPU detector)
- Updating topic remappings to connect different sensors

## See Also

- [Autonomy Bringup](../autonomy/autonomy_bringup/README.md)
- [Wire Perception Package](../autonomy/2_perception/wire_perception/README.md)
- [Wire Approach Planner](../autonomy/3_local/b_planners/wire_approach_planner/README.md)
- [Behavior Executive](../autonomy/5_behavior/behavior_executive/README.md)
