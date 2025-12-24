// SPDX-License-Identifier: MIT

#pragma once

#include <vector>
#include <cmath>
#include <stdexcept>

namespace laser_scan_normalizer {

/**
 * @brief 2D点を表す構造体
 */
struct Point2D {
  double x;
  double y;

  Point2D() : x(0.0), y(0.0) {}
  Point2D(double x, double y) : x(x), y(y) {}
};

/**
 * @brief スキャン点群をリサンプリングするクラス
 *
 * 点群を均等な距離間隔にリサンプリングします。
 * - 点間隔が小さい場合は間引き
 * - 点間隔が適切な場合は補間点を生成
 * - 点間隔が大きすぎる場合はそのまま保持（不連続点の検出）
 */
class ScanPointResampler {
public:
  /**
   * @brief コンストラクタ
   * @param dthreS 目標点間隔 [m] (デフォルト: 0.05m = 5cm)
   * @param dthreL 最大点間隔 [m] (デフォルト: 0.25m = 25cm)
   */
  ScanPointResampler(double dthreS = 0.05, double dthreL = 0.25)
    : dthreS_(dthreS), dthreL_(dthreL), dis_(0.0) {
    validateParameters();
  }

  /**
   * @brief パラメータを設定
   * @param distanceThreshold 目標点間隔 [m]
   * @param lengthThreshold 最大点間隔 [m]
   */
  void setParameters(double distanceThreshold, double lengthThreshold) {
    dthreS_ = distanceThreshold;
    dthreL_ = lengthThreshold;
    validateParameters();
  }

  /**
   * @brief 点群をリサンプリング
   * @param points 入力点群（リサンプリング後に更新される）
   */
  void resamplePoints(std::vector<Point2D>& points) {
    if (points.empty()) {
      return;
    }

    std::vector<Point2D> newPoints;
    newPoints.reserve(points.size());

    // 累積距離の初期化
    dis_ = 0.0;

    // 最初の点は必ず追加
    Point2D prevPoint = points[0];
    newPoints.push_back(prevPoint);

    // 2点目以降の処理
    for (size_t i = 1; i < points.size(); i++) {
      const Point2D& currentPoint = points[i];
      Point2D newPoint;
      bool inserted = false;

      if (findInterpolatePoint(currentPoint, prevPoint, newPoint, inserted)) {
        newPoints.push_back(newPoint);
        prevPoint = newPoint;
        dis_ = 0.0;

        if (inserted) {
          i--;  // 現在の点をもう一度処理
          continue;
        }
      } else {
        prevPoint = currentPoint;
      }
    }

    points = std::move(newPoints);
  }

  // Getter
  double getDistanceThreshold() const { return dthreS_; }
  double getLengthThreshold() const { return dthreL_; }

private:
  double dthreS_;  // 目標点間隔 [m]
  double dthreL_;  // 最大点間隔 [m]
  double dis_;     // 累積距離

  void validateParameters() {
    if (dthreS_ <= 0.0) {
      throw std::invalid_argument("Distance threshold must be positive");
    }
    if (dthreL_ <= dthreS_) {
      throw std::invalid_argument("Length threshold must be greater than distance threshold");
    }
  }

  /**
   * @brief 補間点を探索
   * @param cp 現在の点
   * @param pp 前の点
   * @param np 新しい点（出力）
   * @param inserted 補間点が挿入されたかどうか（出力）
   * @return 点を追加すべき場合はtrue
   */
  bool findInterpolatePoint(const Point2D& cp, const Point2D& pp,
                            Point2D& np, bool& inserted) {
    inserted = false;

    // 2点間の距離を計算
    double dx = cp.x - pp.x;
    double dy = cp.y - pp.y;
    double L = std::sqrt(dx * dx + dy * dy);

    // 点が重なっている場合は処理しない
    if (L < 1e-6) {
      return false;
    }

    // Case 1: 累積距離 + 現在の距離が閾値未満
    if (dis_ + L < dthreS_) {
      dis_ += L;
      return false;
    }

    // Case 2: 累積距離 + 現在の距離が最大閾値以上
    if (dis_ + L >= dthreL_) {
      np = cp;
      return true;
    }

    // Case 3: 補間点を生成
    double ratio = (dthreS_ - dis_) / L;
    if (ratio < 0.0 || ratio > 1.0) {
      np = cp;
      return true;
    }

    np.x = pp.x + dx * ratio;
    np.y = pp.y + dy * ratio;
    inserted = true;

    return true;
  }
};

}  // namespace laser_scan_normalizer
