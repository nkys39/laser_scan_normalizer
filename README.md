# laser_scan_normalizer

A ROS1/ROS2 compatible package for processing and normalizing LaserScan messages.

## Overview

This package provides a node that subscribes to `sensor_msgs/LaserScan` messages, applies processing/normalization, and publishes the result.

## Topics

| Name | Type | Description |
|------|------|-------------|
| `scan` (input) | sensor_msgs/LaserScan | Input laser scan |
| `scan_normalized` (output) | sensor_msgs/LaserScan | Processed laser scan |

## Usage

### ROS1

```bash
rosrun laser_scan_normalizer laser_scan_normalizer_node
```

### ROS2 (Standalone Node)

```bash
ros2 run laser_scan_normalizer laser_scan_normalizer_node
```

### ROS2 (Component)

```bash
ros2 component standalone laser_scan_normalizer laser_scan_normalizer::LaserScanNormalizerROS2
```

## Build

### ROS1

```bash
cd ~/catkin_ws/src
git clone <this_repository>
cd ..
catkin_make
```

### ROS2

```bash
cd ~/ros2_ws/src
git clone <this_repository>
cd ..
colcon build --packages-select laser_scan_normalizer
```

## License

MIT License
