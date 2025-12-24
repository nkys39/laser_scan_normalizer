# ScanPointResampler 設計ドキュメント

## 概要

ScanPointResamplerは、レーザースキャンの点群を均等な間隔にリサンプリングするフィルターです。
点群の密度を正規化することで、スキャンマッチングなどの後処理の安定性を向上させます。

## 機能

- 点群を指定した距離間隔でリサンプリング
- 点間隔が小さい場合は間引き
- 点間隔が大きい場合は補間点を生成（オプション）

## アルゴリズム

### 処理フロー

```
入力: 元のスキャン点群
  ↓
累積距離を計算しながら点を走査
  ↓
┌─────────────────────────────────────────────────┐
│ Case 1: 累積距離 + 点間距離 < dthreS (目標間隔)  │
│   → 累積距離を加算して次の点へ（点を追加しない） │
├─────────────────────────────────────────────────┤
│ Case 2: 累積距離 + 点間距離 >= dthreL (最大間隔) │
│   → 現在の点をそのまま追加（補間しない）         │
├─────────────────────────────────────────────────┤
│ Case 3: dthreS <= 累積距離 + 点間距離 < dthreL   │
│   → 補間点を計算して追加（enable_interpolation時）│
└─────────────────────────────────────────────────┘
  ↓
出力: リサンプリング済み点群
```

## パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `enable_resampling` | true | リサンプリングの有効/無効 |
| `enable_interpolation` | false | 補間の有効/無効 |
| `resampler_distance_threshold` (dthreS) | 0.05 m (5cm) | リサンプリング後の目標点間隔 |
| `resampler_length_threshold` (dthreL) | 0.25 m (25cm) | 最大点間隔（これ以上離れた点は補間せず保持） |

### パラメータの関係

- `dthreL > dthreS` である必要がある
- `dthreS`が小さいほど密な点群になる
- `dthreL`は不連続な領域（物体の境界など）を検出する閾値として機能

## リサンプリングモード

### 間引きのみ (`enable_interpolation: false`) - デフォルト

```
入力: sensor_msgs/LaserScan
  ↓
1. ranges配列を直接走査
2. 隣接点間の距離を余弦定理で計算
   L = sqrt(r1^2 + r2^2 - 2*r1*r2*cos(angle_diff))
3. 累積距離がdthreS未満の点をinfinityで無効化（間引き）
  ↓
出力: sensor_msgs/LaserScan（構造維持、一部rangeがinfinity）
```

**特徴:**
- LaserScanの構造（angle_min, angle_max, angle_increment）を維持
- 間引きのみ（補間は不可）
- オーバーヘッドが少ない
- intensitiesとの対応関係を維持可能

### 補間あり (`enable_interpolation: true`)

```
入力: sensor_msgs/LaserScan
  ↓
1. LaserScan → XY座標変換
   x[i] = ranges[i] * cos(angle_min + i * angle_increment)
   y[i] = ranges[i] * sin(angle_min + i * angle_increment)
  ↓
2. XY座標でリサンプリング（元のアルゴリズム適用）
   - 累積距離ベースで点を間引き/補間
  ↓
3. 元グリッドに再マッピング
   - リサンプリング後の各点の角度を計算
   - 最近傍のグリッドインデックスにrange値を設定
   - 他のインデックスはinfinityに設定
  ↓
出力: sensor_msgs/LaserScan（構造維持、歪みなし）
```

**特徴:**
- 補間点の生成が可能
- 元のangle_incrementグリッドを維持（歪みなし）
- 座標変換のオーバーヘッドあり
- intensitiesは破棄される

### モードの比較

| 項目 | 間引きのみ (false) | 補間あり (true) |
|------|-------------------|-----------------|
| 補間 | × 不可 | ○ 可能 |
| 間引き | ○ 可能 | ○ 可能 |
| LaserScan構造 | ○ 維持 | ○ 維持 |
| 歪み | なし | なし |
| オーバーヘッド | 小 | 大 |
| intensities | ○ 維持可能 | × 破棄 |
| 用途 | 高速フィルタリング | 高精度リサンプリング |

## 実装ファイル

```
include/laser_scan_normalizer/
├── laser_scan_normalizer.hpp      # LaserScanProcessor
└── scan_point_resampler.hpp       # ScanPointResampler（XY座標用）

src/
├── laser_scan_normalizer_ros1.cpp # ROS1ノード
└── laser_scan_normalizer_ros2.cpp # ROS2ノード
```

## 使用例

### ROS1

```bash
# 間引きのみ（デフォルト）
rosrun laser_scan_normalizer laser_scan_normalizer_node

# 補間あり
rosrun laser_scan_normalizer laser_scan_normalizer_node _enable_interpolation:=true

# パラメータ変更
rosrun laser_scan_normalizer laser_scan_normalizer_node \
  _enable_interpolation:=true \
  _resampler_distance_threshold:=0.03 \
  _resampler_length_threshold:=0.20
```

### ROS2

```bash
# 間引きのみ（デフォルト）
ros2 run laser_scan_normalizer laser_scan_normalizer_node

# 補間あり
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args -p enable_interpolation:=true

# パラメータ変更
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args \
  -p enable_interpolation:=true \
  -p resampler_distance_threshold:=0.03 \
  -p resampler_length_threshold:=0.20
```

## 参考

- 元のコード: `es_slam/scan_point_resampler.hpp`
- 用途: SLAM前処理としての点群正規化
