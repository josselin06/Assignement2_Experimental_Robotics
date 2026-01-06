# Assignment 2 – Experimental Robotics (package “my_opencv”)

This repository contains only the ROS 2 package **my_opencv**, extracted from our full workspace used in the Experimental Robotics course.

In the original Docker container for the assignment, the workspace `/root/ws/src` contains several packages (course material). Since these packages come from the provided resources, we only version **my_opencv** here, and in particular the files corresponding to our own work for this assignment.

## Objective

This assignment implements a planning-based ArUco mission using PlanSys2 + Nav2:

1. The robot visits 4 predefined waypoints (`wp1..wp4`) and detects ArUco markers.
2. Detected IDs are stored persistently in `/tmp/aruco_ids.txt`.
3. Each new ID is also associated to a waypoint pose in `/tmp/aruco_id2pose.txt`.
4. Then, repeatedly, the robot navigates to the stored pose of the next unprocessed marker ID,
   rotates to find it if needed, centers it in the camera, and publishes an annotated image on
   `/camera/marker_selected`. Processed IDs are stored in `/tmp/aruco_done.txt`.

## Important files for evaluation

- `src/explore_action.cpp`  
  PlanSys2 action **explore**: Nav2 navigation to waypoints + scan window + ArUco detection + persistence.
  Includes an optional “approach search” around the waypoint when no marker is detected.

- `src/take_picture_next_action.cpp`  
  PlanSys2 action **take_picture_next**: selects the next marker ID not in the done list, navigates to its stored pose,
  rotates in place if the marker is not visible, centers the marker, publishes `/camera/marker_selected`, and marks it as done.

- `pddl/domain.pddl` and `pddl/problem.pddl`  
  Planning model for the mission (explore all waypoints, then take pictures until a max counter is reached).

- `launch/assignment2_plansys.launch.py`  
  Launches PlanSys2 bringup (distributed) and both action nodes.

## How it works (high-level)

- The two PlanSys2 actions coordinate using simple text files in `/tmp`:
  - `/tmp/aruco_ids.txt` (detected IDs)
  - `/tmp/aruco_id2pose.txt` (ID -> pose mapping)
  - `/tmp/aruco_done.txt` (processed IDs)

- The planning domain uses two counters:
  - `ec` ensures all `explore` actions are executed first (4 waypoints).
  - `pc` ensures `take_picture_next` is executed a fixed number of times after exploration.

## Build & run

Inside the provided ROS 2 workspace:

```bash
colcon build --packages-select my_opencv
source install/setup.bash
and then in 5 terminals: 

source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py rviz:=false gz_args:="-s"

source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  autostart:=True \
  slam:=False \
  use_localization:=False \
  use_composition:=False \
  params_file:=/root/ws/src/my_opencv/config/nav2_params.yaml  

source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
ros2 launch my_opencv assignment2_plansys.launch.py use_sim_time:=True

source /opt/ros/jazzy/setup.bash 
source ~/ws/install/setup.bash 
ros2 run plansys2_terminal plansys2_terminal 
# then: get plan run

