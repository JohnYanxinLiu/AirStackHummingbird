# Ground Control Station Bringup
A simple tool to aggregate **VDB MAPPING results** and **ROBOT POSES**.
## ROS2 Topics
### Subscription:
- `$(ROBOT_NAME)/vdb_mapping/vdb_map_visualization`: vdb_mapping results
- `$(ROBOT_NAME)/odometry_conversion/odometry`: Odometry data w.r.t. each robot’s map frame.
- `$(ROBOT_NAME)/global_pose`: Odometry data w.r.t global frame.
### Publication:
- `/robot_team/pose`: A set of robot poses
- `/robot_team/point_cloud`: Aggregated VDB mapping results
## User Parameters
- `mask_size_x`: Mask voxels by `mask_size_x` around the robots
- `mask_size_y`: Mask voxels by `mask_size_y` around the robots
- `mask_size_z`: Mask voxels by `mask_size_z` around the robots

### Contact: yunwool@andrew.cmu.edu