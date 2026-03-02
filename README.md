# localization_utils

Utility ROS 2 nodes used by the localization bringup pipeline.

This package provides three executables:
- `pose_estimate_remapper`
- `static_odom_publisher`
- `tf_height_remover`

## Build and run

```bash
colcon build --symlink-install --packages-select localization_utils
```

Run any node with:

```bash
ros2 run localization_utils <executable_name>
```

---

## 1) pose_estimate_remapper.py

### Purpose
Bridges Nav2 initial pose input to FAST-LIO localizer relocalization service.

When a pose is published to `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`), this node:
1. Extracts position and quaternion
2. Converts quaternion to roll/pitch/yaw
3. Calls `/localizer/relocalize` (`interface/srv/Relocalize`) with:
	 - `pcd_path`
	 - `x, y, z`
	 - `yaw, pitch, roll`

### I/O
- **Subscribes:** `/initialpose`
- **Calls service:** `/localizer/relocalize`

### Parameters
- `map_path` (string, default: `maps/map.pcd`): PCD map path passed to relocalizer
- `verbose` (bool, default: `true`): logs requests and service responses

### Example
```bash
ros2 run localization_utils pose_estimate_remapper \
	--ros-args -p map_path:=mtr_maps/c1_processed.pcd -p verbose:=true
```

---

## 2) static_odom_publisher.py

### Purpose
Publishes a periodic odometry message with fixed zero translation and zero quaternion fields.

This is used as a helper topic for localization workflows that require a steady odometry stream.

### I/O
- **Publishes:** configurable odometry topic (default `/static_odom`)

### Parameters
- `output_topic` (string, default: `/static_odom`)
- `parent_frame` (string, default: `/map`)
- `child_frame` (string, default: `/static_odom`)
- `period` (double, default: `0.5` seconds)
- `verbose` (bool, default: `false`)

### Example
```bash
ros2 run localization_utils static_odom_publisher \
	--ros-args \
	-p output_topic:=/static_odom \
	-p parent_frame:=map \
	-p child_frame:=static_odom \
	-p period:=0.2
```

---

## 3) tf_height_remover.py

### Purpose
Creates a derived TF frame that removes vertical displacement from an input frame.

At 10 Hz, this node:
1. Looks up transform `world_frame -> input_frame`
2. Reads `z` from that transform
3. Broadcasts `input_frame -> output_frame` with:
	 - `x = 0`, `y = 0`
	 - `z = -z - z_extra_offset`
	 - identity rotation

This is commonly used to create a footprint-like frame with leveled height for navigation.

### I/O
- **Reads TF:** `world_frame -> input_frame`
- **Broadcasts TF:** `input_frame -> output_frame`

### Parameters
- `world_frame` (string, default: `map`)
- `input_frame` (string, default: `body`)
- `output_frame` (string, default: `base_footprint`)
- `z_extra_offset` (double, default: `0.0`)
- `verbose` (bool, default: `true`)

### Example
```bash
ros2 run localization_utils tf_height_remover \
	--ros-args \
	-p world_frame:=map \
	-p input_frame:=body \
	-p output_frame:=base_footprint \
	-p z_extra_offset:=0.0 \
	-p verbose:=false
```

---

## Notes

- `pose_estimate_remapper` depends on FAST-LIO localizer service interfaces (`interface/srv/Relocalize`).
- `tf_height_remover` uses a TF buffer with 10-second cache time.
- In integrated bringup, these nodes are typically launched by `clio_bringup` launch files rather than manually.
