# clio_bringup

ROS 2 bringup package for Innowing Clio Guide Robot Project mapping, localization, and navigation workflows.

## Overview

`clio_bringup` orchestrates:
- FAST-LIO odometry/mapping
- Localization against a saved `.pcd` map
- Nav2 navigation with a 2D occupancy map

## Launch Files

- `mapping.launch.py`  
  Runs mapping pipeline (driver + FAST-LIO, optional bag recording).

- `localization.launch.py`  
  Runs localization-only flow (localizer + pose remapper, optional driver/FAST-LIO/bag).

- `navigation.launch.py`  
  Runs full navigation stack (FAST-LIO + localizer + Nav2 + robot bridge).

- `register_localization.launch.py`  
  Integrated startup for relocalization + Nav2 with conditional components.

## Prerequisites

TODO: Confirm prerequisites

- ROS 2 Humble
- Workspace built with `colcon`
- Dependencies installed:
  - `livox_ros_driver2`
  - FAST-LIO stack
  - `localizer`
  - `guide_robot_localization`
  - Nav2


## Quick Start

### 1) Mapping (live sensor)

```bash
ros2 launch clio_bringup mapping.launch.py
```

### 2) Localization (live sensor)

```bash
ros2 launch clio_bringup register_localization.launch.py \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml]
```

### 3) Localization (rosbag)

```bash
ros2 launch clio_bringup register_localization.launch.py \
  driver:=false \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml] \
  use_sim_time:=True
```

In another terminal - 
```bash
ros2 bag play [path to your rosbag] --clock
```

### 4) Navigation (live sensor)

```bash
ros2 launch clio_bringup navigation.launch.py \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml]
```

## Launch Arguments

Please go to the respective package repositories for details

## Related Repositories

Check out these repositories as well, they are also used in this project: 
- https://github.com/Tranxpotter/2d-map-tools
- https://github.com/Tranxpotter/pcd-map-preprocessing

## Maintainer

- Name: `<maintainer_name>`
- Email: `<maintainer_email>`