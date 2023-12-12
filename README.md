# scout2_2d_slam_module

This module enables the Silvanus robot to estimate its odometry using [Direct Lidar Inertial Odometry](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) and execute SLAM using [GMapping](https://wiki.ros.org/gmapping). The module is completely containerized and can easily adapted to work with other robots because of Docker.

The scout_2d_slam_module is used in the [ball_picking_project repository](https://github.com/IFRoS-ELTE/ball_picking_project).
<br><br>
**Created by:** <br>
Kevin Schmidt: cbsx3q@inf.elte.hu

---

## Requirements

- Docker (engine)
- Docker Compose

Specific requirements and dependencies for the included ROS packages can be found in the Dockerfile. For more information about the containerization approach, refer to [this page](https://github.com/IFRoS-ELTE/ball_picking_project/blob/main/docs/docker.md).

## Setup for stand-alone use

### 1. Clone this repository recursively on the Scout2.0

```bash
git clone --recurse-submodules https://github.com/IFRoS-ELTE/scout2_2d_slam_module.git
```

### 2. Build and launch the containers

```bash
cd ./scout2_2d_slam_module/docker && sudo docker compose up
```

### Stopping and re-launching

To stop and close the nodes, simply use `CTRL+C`, this will stop all containers. You can use `sudo docker compose up` if you want to re-launch. If you, for some reason, want to remove the containers, you can do so with the `sudo docker compose down` command. This means that at the next startup using `sudo docker compose up`, the containers will be newly created and all the ros workspaces inside will have to be freshly built as well.

## Submodules

### [direct_lidar_inertial_odometry](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)

Repository used to calculate robot odometry based on LiDAR and and IMU data.
