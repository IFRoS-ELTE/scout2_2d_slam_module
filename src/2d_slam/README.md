# 2d_slam Package

This package contains a ROS (Robot Operating System) some helper nodes and launch files to run GMapping with the Silvanus Robot.

## Prerequisites / Installation / Usage

This package should be run only as part of the scout2_2d_slam_module. All requisites are listed in the Dockerfile of this repository. For further instructions, refer to the README of this repository.

## Nodes

### modify_laserscan_frame_id_node

This node subscribes to a 2D laser scan topic and alters the frame_id to a specified string. It then publishes the modified laser scan message.

The node is needed because there was no other way to easily change the frame id of the laser scan.

#### Input parameters

| Name         | Description                                                                     |
| ------------ | ------------------------------------------------------------------------------- |
| input_topic  | The laser scan topic the node should subscribe to                               |
| output_topic | The name of the topic the node should publish the altered laser scan message on |
| new_frame_id | the desired name of the new frame id of the laser scan message                  |

### z_drift_fixer_node

This node subscribes tf and grabs a desired frame, sets the z_translation to a desired value and then republishes that frame under a new frame id which can be selected by the user.

This node was necessary because the drift in z direction in the odometry interferes with the process of mapping the environment. Therefore, a dirty fix was to simply set the z translation to a static value.

#### Input parameters

| Name               | Description                                                 |
| ------------------ | ----------------------------------------------------------- |
| original_frame_id  | The ID of the frame that should be altered                  |
| corrected_frame_id | The desired name that should be used for the modified frame |
| z_translation      | The fixed z translation value in meters                     |

## Launch files

### mapping_pipeline.launch

This is the main launch file for the scout2_2d_slam module for the Silvanus robot.

#### Launched nodes

- **direct_lidar_inertial_odometry**
  - refer to the documentation in the DLIO submodule which is also located in the src directory of this repository.
- **rviz**
  - Launches RviZ with the configuration that includes all the necessary visualizations for the autonomous mapping and exploration with the Silvanus robot
- **pointcloud_to_laserscan**
  - Please refer to the [official ROS documentation](<[Title](https://wiki.ros.org/pointcloud_to_laserscan)>) for more information and input parameter description
- **lscan_republish_node**
  - Please refer to the "modify_laserscan_frame_id_node" description above for a description of the input parameters
- **z_coordinate_fixer_node**
  - - Please refer to the "z_coordinate_fixer_node" description above for a description of the input parameters
- **gmapping_node**
  - Please refer to the [official ROS documentation](https://wiki.ros.org/pointcloud_to_laserscan) for more information and input parameter description
