# ScanPointResampler 設計ドキュメント

## 概要

ScanPointResamplerは、レーザースキャンの点群を均等な間隔にリサンプリングするフィルターです。
点群の密度を正規化することで、スキャンマッチングなどの後処理の安定性を向上させます。

## 機能

- 点群を指定した距離間隔でリサンプリング
- 点間隔が大きい場合は補間点を生成
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
│   → 補間点を計算して追加                         │
└─────────────────────────────────────────────────┘
  ↓
出力: リサンプリング済み点群
```

### 補間点の計算

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
| `dthreS` | 0.05 m (5cm) | リサンプリング後の目標点間隔 |
| `dthreL` | 0.25 m (25cm) | 最大点間隔（これ以上離れた点は補間せず保持） |

### パラメータの関係

- `dthreL > dthreS` である必要がある
- `dthreS`が小さいほど密な点群になる
- `dthreL`は不連続な領域（物体の境界など）を検出する閾値として機能

## 実装方針

### 1. データ型の変換

元のコードは独自の`Scan2D`/`LPoint2D`型を使用しているため、`sensor_msgs/LaserScan`用に変換が必要。

**元のコード:**
```cpp
struct LPoint2D {
    int sid;      // スキャンID
    double x, y;  // XY座標
};

struct Scan2D {
    std::vector<LPoint2D> lps;
};
```

**LaserScanでの対応:**
- LaserScanは極座標形式（angle, range）
- XY座標への変換が必要: `x = range * cos(angle)`, `y = range * sin(angle)`
- または直接range配列を処理する方式も検討

### 2. 実装アプローチの選択肢

#### 方式A: XY座標変換方式

```cpp
// LaserScan → XY座標に変換 → リサンプリング → LaserScan に戻す
```

- メリット: 元のアルゴリズムをほぼそのまま適用可能
- デメリット: 座標変換のオーバーヘッド、角度情報の再計算が必要

#### 方式B: 極座標直接処理方式（推奨）

```cpp
// range配列を直接処理（角度方向の隣接点間距離でリサンプリング）
```

- メリット: 変換オーバーヘッドなし、LaserScanの構造を維持
- デメリット: アルゴリズムの一部修正が必要

### 3. 推奨実装方針

**方式B（極座標直接処理）を推奨**

理由:
- LaserScanは等角度間隔のデータ構造であり、出力も同じ構造を維持すべき
- リサンプリングの目的（点群密度の正規化）は、range値のフィルタリングで達成可能
- パフォーマンスが良い

### 4. 実装箇所

```
include/laser_scan_normalizer/
├── laser_scan_normalizer.hpp      # LaserScanProcessor に処理を追加
└── scan_point_resampler.hpp       # (新規) リサンプラークラス

src/
├── laser_scan_normalizer_ros1.cpp # パラメータ読み込み追加
└── laser_scan_normalizer_ros2.cpp # パラメータ読み込み追加
```

### 5. インターフェース案

```cpp
class ScanPointResampler {
public:
    ScanPointResampler(double dthreS = 0.05, double dthreL = 0.25);

    void setParameters(double distanceThreshold, double lengthThreshold);

    // LaserScan用のリサンプリング
    void resample(sensor_msgs::LaserScan& scan);

private:
    double dthreS_;  // 目標点間隔 [m]
    double dthreL_;  // 最大点間隔 [m]

    // 2点間の距離計算（極座標）
    double calcDistance(double range1, double range2, double angle_diff);
};
```

### 6. ROSパラメータ

```yaml
# パラメータ例
laser_scan_normalizer:
  ros__parameters:
    resampler_distance_threshold: 0.05  # dthreS [m]
    resampler_length_threshold: 0.25    # dthreL [m]
    enable_resampling: true
```

## 次のステップ

1. `scan_point_resampler.hpp`の作成
2. `LaserScanProcessor`への統合
3. ROS1/ROS2パラメータ対応
4. テストの作成

## 参考

- 元のコード: `es_slam/scan_point_resampler.hpp`
- 用途: SLAM前処理としての点群正規化
