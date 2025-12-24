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

## パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `enable_resampling` | bool | true | リサンプリングの有効/無効 |
| `enable_interpolation` | bool | false | 補間の有効/無効 |
| `resampler_distance_threshold` | double | 0.05 | 目標点間隔 [m] |
| `resampler_length_threshold` | double | 0.25 | 最大点間隔 [m] |

### 補間モード

| enable_interpolation | 動作 | 特徴 |
|---------------------|------|------|
| false（デフォルト） | 間引きのみ | 高速、LaserScan構造を維持 |
| true | 間引き＋補間 | 点群密度を均一化、元グリッドに再マッピング |

### パラメータ指定例

#### ROS1

```bash
# 間引きのみ（デフォルト）
rosrun laser_scan_normalizer laser_scan_normalizer_node _resampler_distance_threshold:=0.03

# 補間あり
rosrun laser_scan_normalizer laser_scan_normalizer_node _enable_interpolation:=true
```

#### ROS2

```bash
# 間引きのみ（デフォルト）
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args -p resampler_distance_threshold:=0.03

# 補間あり
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args -p enable_interpolation:=true
```

## トピックのリマップ

### ROS1

```bash
rosrun laser_scan_normalizer laser_scan_normalizer_node scan:=/lidar/scan scan_normalized:=/lidar/scan_normalized
```

### ROS2

```bash
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args -r scan:=/lidar/scan -r scan_normalized:=/lidar/scan_normalized
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
