# laser_scan_normalizer

ROS1/ROS2両対応のLaserScanメッセージ処理・正規化パッケージです。

## 概要

`sensor_msgs/LaserScan`メッセージを購読し、処理・正規化を行った結果を出力するノードを提供します。

## トピック

| 名前 | 型 | 説明 |
|------|------|-------------|
| `scan` (入力) | sensor_msgs/LaserScan | 入力レーザースキャン |
| `scan_normalized` (出力) | sensor_msgs/LaserScan | 処理後のレーザースキャン |

## 使い方

### ROS1

```bash
rosrun laser_scan_normalizer laser_scan_normalizer_node
```

### ROS2 (スタンドアロンノード)

```bash
ros2 run laser_scan_normalizer laser_scan_normalizer_node
```

### ROS2 (コンポーネント)

```bash
ros2 component standalone laser_scan_normalizer laser_scan_normalizer::LaserScanNormalizerROS2
```

## ビルド方法

### ROS1

```bash
cd ~/catkin_ws
catkin_make
```

### ROS2

```bash
cd ~/ros2_ws
colcon build --packages-select laser_scan_normalizer
```

## ライセンス

MIT License
