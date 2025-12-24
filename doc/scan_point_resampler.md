# ScanPointResampler 設計ドキュメント

## 概要

ScanPointResamplerは、レーザースキャンの点群を均等な間隔にリサンプリングするフィルターです。
点群の密度を正規化することで、スキャンマッチングなどの後処理の安定性を向上させます。

## 機能

- 点群を指定した距離間隔でリサンプリング
- 点間隔が大きい場合は補間点を生成（方式Aのみ）
- 点間隔が小さい場合は間引き

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
│   → 補間点を計算して追加（方式Aのみ）            │
└─────────────────────────────────────────────────┘
  ↓
出力: リサンプリング済み点群
```

### 補間点の計算（方式Aのみ）

2点間(pp → cp)で補間点を生成する場合:

```
ratio = (dthreS - dis) / L

np.x = pp.x + dx * ratio
np.y = pp.y + dy * ratio
```

- `dis`: 累積距離
- `L`: 2点間の距離
- `dthreS`: 目標点間隔

## パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `enable_resampling` | true | リサンプリングの有効/無効 |
| `resampling_method` | "xy" | リサンプリング方式 ("xy" or "polar") |
| `resampler_distance_threshold` (dthreS) | 0.05 m (5cm) | リサンプリング後の目標点間隔 |
| `resampler_length_threshold` (dthreL) | 0.25 m (25cm) | 最大点間隔（これ以上離れた点は補間せず保持） |

### パラメータの関係

- `dthreL > dthreS` である必要がある
- `dthreS`が小さいほど密な点群になる
- `dthreL`は不連続な領域（物体の境界など）を検出する閾値として機能

## リサンプリング方式

### 方式A: XY座標変換方式 (`resampling_method: "xy"`)

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
3. XY座標 → LaserScan変換
   - リサンプリング後の点数に応じてangle_incrementを再計算
   - range[i] = sqrt(x[i]^2 + y[i]^2)
   - angle[i] = atan2(y[i], x[i])
  ↓
出力: sensor_msgs/LaserScan（点数が変化する可能性あり）
```

**特徴:**
- 元のアルゴリズムを忠実に再現
- 補間点の生成が可能
- 座標変換のオーバーヘッドあり
- 出力のLaserScanは点数が変化する可能性あり

### 方式B: 極座標直接処理方式 (`resampling_method: "polar"`)

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

### 方式の比較

| 項目 | 方式A (xy) | 方式B (polar) |
|------|-----------|---------------|
| 補間 | ○ 可能 | × 不可 |
| 間引き | ○ 可能 | ○ 可能 |
| LaserScan構造維持 | × 変化する | ○ 維持 |
| オーバーヘッド | 大 | 小 |
| intensities | × 破棄 | ○ 維持可能 |
| 用途 | 高精度リサンプリング | 軽量フィルタリング |

## 実装ファイル

```
include/laser_scan_normalizer/
├── laser_scan_normalizer.hpp      # LaserScanProcessor（方式A/B両対応）
└── scan_point_resampler.hpp       # ScanPointResampler（XY座標用）

src/
├── laser_scan_normalizer_ros1.cpp # ROS1ノード（パラメータ対応）
└── laser_scan_normalizer_ros2.cpp # ROS2ノード（パラメータ対応）
```

## 使用例

### ROS1

```bash
# 方式A（デフォルト）
rosrun laser_scan_normalizer laser_scan_normalizer_node

# 方式B
rosrun laser_scan_normalizer laser_scan_normalizer_node _resampling_method:=polar

# パラメータ変更
rosrun laser_scan_normalizer laser_scan_normalizer_node \
  _resampling_method:=xy \
  _resampler_distance_threshold:=0.03 \
  _resampler_length_threshold:=0.20
```

### ROS2

```bash
# 方式A（デフォルト）
ros2 run laser_scan_normalizer laser_scan_normalizer_node

# 方式B
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args -p resampling_method:=polar

# パラメータ変更
ros2 run laser_scan_normalizer laser_scan_normalizer_node --ros-args \
  -p resampling_method:=xy \
  -p resampler_distance_threshold:=0.03 \
  -p resampler_length_threshold:=0.20
```

## 参考

- 元のコード: `es_slam/scan_point_resampler.hpp`
- 用途: SLAM前処理としての点群正規化
