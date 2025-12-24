// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <limits>

#ifdef ROS1
#include <sensor_msgs/LaserScan.h>
#else
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <laser_scan_normalizer/scan_point_resampler.hpp>

namespace laser_scan_normalizer {

#ifdef ROS1
using LaserScan = sensor_msgs::LaserScan;
#else
using LaserScan = sensor_msgs::msg::LaserScan;
#endif

/**
 * @brief リサンプリング方式
 */
enum class ResamplingMethod {
  XY,    // 方式A: XY座標変換方式（補間あり）
  POLAR  // 方式B: 極座標直接処理方式（間引きのみ）
};

/**
 * @brief LaserScan processing utility class
 *
 * This class provides common processing functions for LaserScan messages.
 * It can be used with both ROS1 and ROS2.
 */
class LaserScanProcessor {
public:
  LaserScanProcessor()
    : enable_resampling_(true)
    , resampling_method_(ResamplingMethod::XY)
    , dthreS_(0.05)
    , dthreL_(0.25)
    , resampler_(0.05, 0.25) {}

  /**
   * @brief リサンプリングの有効/無効を設定
   */
  void setEnableResampling(bool enable) {
    enable_resampling_ = enable;
  }

  /**
   * @brief リサンプリング方式を設定
   * @param method リサンプリング方式
   */
  void setResamplingMethod(ResamplingMethod method) {
    resampling_method_ = method;
  }

  /**
   * @brief リサンプリング方式を文字列で設定
   * @param method "xy" or "polar"
   */
  void setResamplingMethod(const std::string& method) {
    if (method == "polar") {
      resampling_method_ = ResamplingMethod::POLAR;
    } else {
      resampling_method_ = ResamplingMethod::XY;
    }
  }

  /**
   * @brief リサンプラーのパラメータを設定
   * @param dthreS 目標点間隔 [m]
   * @param dthreL 最大点間隔 [m]
   */
  void setResamplerParameters(double dthreS, double dthreL) {
    dthreS_ = dthreS;
    dthreL_ = dthreL;
    resampler_.setParameters(dthreS, dthreL);
  }

  /**
   * @brief Process a LaserScan message
   *
   * @param input Input LaserScan message
   * @return Processed LaserScan message
   */
  LaserScan process(const LaserScan& input) const {
    LaserScan output = input;

    // Step 1: Clamp range values
    for (size_t i = 0; i < output.ranges.size(); ++i) {
      float range = output.ranges[i];

      if (!std::isfinite(range)) {
        continue;
      }

      if (range < output.range_min) {
        output.ranges[i] = output.range_min;
      } else if (range > output.range_max) {
        output.ranges[i] = output.range_max;
      }
    }

    // Step 2: Resample points if enabled
    if (enable_resampling_) {
      if (resampling_method_ == ResamplingMethod::XY) {
        output = resampleScanXY(output);
      } else {
        output = resampleScanPolar(output);
      }
    }

    return output;
  }

private:
  bool enable_resampling_;
  ResamplingMethod resampling_method_;
  double dthreS_;
  double dthreL_;
  mutable ScanPointResampler resampler_;

  /**
   * @brief 極座標での2点間距離を計算（余弦定理）
   */
  double calcPolarDistance(double r1, double r2, double angle_diff) const {
    return std::sqrt(r1 * r1 + r2 * r2 - 2.0 * r1 * r2 * std::cos(angle_diff));
  }

  /**
   * @brief 方式B: 極座標直接処理方式（間引きのみ）
   *
   * LaserScanの構造を維持しながら、近すぎる点を無効化（間引き）する。
   * 補間はできないが、オーバーヘッドが少ない。
   */
  LaserScan resampleScanPolar(const LaserScan& input) const {
    LaserScan output = input;

    if (output.ranges.size() < 2) {
      return output;
    }

    double dis = 0.0;  // 累積距離
    int prevValidIdx = -1;  // 前回の有効な点のインデックス
    double prevRange = 0.0;

    for (size_t i = 0; i < output.ranges.size(); ++i) {
      float range = output.ranges[i];

      // 無効な点はスキップ
      if (!std::isfinite(range) || range < output.range_min || range > output.range_max) {
        continue;
      }

      // 最初の有効な点
      if (prevValidIdx < 0) {
        prevValidIdx = static_cast<int>(i);
        prevRange = range;
        continue;
      }

      // 2点間の角度差
      double angle_diff = (i - prevValidIdx) * output.angle_increment;

      // 2点間の距離（余弦定理）
      double L = calcPolarDistance(prevRange, range, angle_diff);

      // Case 1: 累積距離 + 現在の距離が閾値未満 → 間引き
      if (dis + L < dthreS_) {
        dis += L;
        output.ranges[i] = std::numeric_limits<float>::infinity();  // 無効化
        continue;
      }

      // Case 2 & 3: 点を保持
      dis = 0.0;
      prevValidIdx = static_cast<int>(i);
      prevRange = range;
    }

    return output;
  }

  /**
   * @brief 方式A: XY座標変換方式
   */
  LaserScan resampleScanXY(const LaserScan& input) const {
    // LaserScan → XY座標
    std::vector<Point2D> points = laserScanToPoints(input);

    if (points.size() < 2) {
      return input;
    }

    // リサンプリング
    resampler_.resamplePoints(points);

    // XY座標 → LaserScan
    return pointsToLaserScan(points, input);
  }

  /**
   * @brief LaserScanをXY座標に変換
   */
  std::vector<Point2D> laserScanToPoints(const LaserScan& scan) const {
    std::vector<Point2D> points;
    points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      float range = scan.ranges[i];

      // Skip invalid ranges
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }

      double angle = scan.angle_min + i * scan.angle_increment;
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);
      points.emplace_back(x, y);
    }

    return points;
  }

  /**
   * @brief XY座標をLaserScanに変換
   */
  LaserScan pointsToLaserScan(const std::vector<Point2D>& points,
                               const LaserScan& original) const {
    LaserScan output;
    output.header = original.header;
    output.range_min = original.range_min;
    output.range_max = original.range_max;
    output.scan_time = original.scan_time;
    output.time_increment = original.time_increment;

    if (points.empty()) {
      output.angle_min = original.angle_min;
      output.angle_max = original.angle_max;
      output.angle_increment = original.angle_increment;
      return output;
    }

    // 各点の角度とrangeを計算
    std::vector<std::pair<double, double>> angle_range_pairs;
    angle_range_pairs.reserve(points.size());

    for (const auto& p : points) {
      double range = std::sqrt(p.x * p.x + p.y * p.y);
      double angle = std::atan2(p.y, p.x);
      angle_range_pairs.emplace_back(angle, range);
    }

    // 角度でソート
    std::sort(angle_range_pairs.begin(), angle_range_pairs.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // 角度範囲を設定
    output.angle_min = angle_range_pairs.front().first;
    output.angle_max = angle_range_pairs.back().first;

    if (angle_range_pairs.size() > 1) {
      output.angle_increment = (output.angle_max - output.angle_min) /
                               (angle_range_pairs.size() - 1);
    } else {
      output.angle_increment = original.angle_increment;
    }

    // ranges配列を設定
    output.ranges.reserve(angle_range_pairs.size());
    for (const auto& pair : angle_range_pairs) {
      output.ranges.push_back(static_cast<float>(pair.second));
    }

    // intensitiesがある場合は空にする（リサンプリングで対応が崩れるため）
    output.intensities.clear();

    return output;
  }
};

}  // namespace laser_scan_normalizer
