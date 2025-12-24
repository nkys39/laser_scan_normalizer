# laser_scan_normalizer 設計ドキュメント

## 概要

本パッケージは、[livox_to_pointcloud2](https://github.com/koide3/livox_to_pointcloud2)を参考に、ROS1とROS2の両方に対応したLaserScan処理ノードの雛形として作成されました。

## 設計方針

### ROS1/ROS2両対応の仕組み

1. **環境変数`$ROS_VERSION`による分岐**
   - `ROS_VERSION=1`: ROS1 (catkin) としてビルド
   - `ROS_VERSION=2`: ROS2 (ament) としてビルド

2. **CMakeLists.txtでの条件分岐**
   ```cmake
   if($ENV{ROS_VERSION} EQUAL 1)
     # ROS1ビルド設定
   else()
     # ROS2ビルド設定
   endif()
   ```

3. **package.xmlでの条件付き依存関係**
   ```xml
   <depend condition="$ROS_VERSION == 1">roscpp</depend>
   <depend condition="$ROS_VERSION == 2">rclcpp</depend>
   ```

### ファイル構成

```
laser_scan_normalizer/
├── include/laser_scan_normalizer/
│   ├── laser_scan_normalizer.hpp      # 共通処理クラス（ROS非依存ロジック）
│   └── laser_scan_normalizer_ros2.hpp # ROS2ノードクラス定義
├── src/
│   ├── laser_scan_normalizer_ros1.cpp      # ROS1ノード実装
│   ├── laser_scan_normalizer_ros2.cpp      # ROS2コンポーネント実装
│   └── laser_scan_normalizer_ros2_node.cpp # ROS2スタンドアロンノード
├── doc/
│   └── design.md                      # 本ドキュメント
├── CMakeLists.txt
├── package.xml
└── README.md
```

### クラス設計

#### LaserScanProcessor（共通処理クラス）

- 場所: `include/laser_scan_normalizer/laser_scan_normalizer.hpp`
- ROS1/ROS2共通で使用される処理ロジックを実装
- `#ifdef ROS1`によるメッセージ型の切り替え
- `process()`メソッドに実際の処理を記述

#### LaserScanNormalizerROS1

- 場所: `src/laser_scan_normalizer_ros1.cpp`
- ROS1用のノード実装
- `ros::NodeHandle`を使用

#### LaserScanNormalizerROS2

- 場所: `src/laser_scan_normalizer_ros2.cpp`, `include/.../laser_scan_normalizer_ros2.hpp`
- ROS2用のノード/コンポーネント実装
- `rclcpp::Node`を継承
- `RCLCPP_COMPONENTS_REGISTER_NODE`でコンポーネント登録

## 処理フロー

```
[入力: scan] → LaserScanProcessor::process() → [出力: scan_normalized]
```

## 拡張方法

### 処理ロジックの追加

`LaserScanProcessor::process()`メソッド内に処理を追加してください。

```cpp
LaserScan process(const LaserScan& input) const {
  LaserScan output = input;

  // ここに処理を追加
  for (size_t i = 0; i < output.ranges.size(); ++i) {
    // 例: フィルタリング、正規化など
  }

  return output;
}
```

### パラメータの追加

- ROS1: `pnh_.param<T>("param_name", value, default_value)`
- ROS2: `this->declare_parameter<T>("param_name", default_value)`

## 参考資料

- [livox_to_pointcloud2](https://github.com/koide3/livox_to_pointcloud2) - ROS1/ROS2両対応パッケージの参考実装
- [ROS2 Components](https://docs.ros.org/en/humble/Concepts/About-Composition.html) - ROS2コンポーネントの概要
