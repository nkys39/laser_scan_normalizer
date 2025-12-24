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
 * @brief LaserScan processing utility class
 *
 * This class provides common processing functions for LaserScan messages.
 * It can be used with both ROS1 and ROS2.
 */
class LaserScanProcessor {
public:
  LaserScanProcessor()
    : enable_resampling_(true)
    , enable_interpolation_(false)
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
   * @brief 補間の有効/無効を設定
   * @param enable true: 補間あり（XY変換→リサンプリング→元グリッド再マッピング）
   *               false: 間引きのみ（デフォルト）
   */
  void setEnableInterpolation(bool enable) {
    enable_interpolation_ = enable;
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
      if (enable_interpolation_) {
        output = resampleWithInterpolation(output);
      } else {
        output = resamplePolarOnly(output);
      }
    }

    return output;
  }

private:
  bool enable_resampling_;
  bool enable_interpolation_;
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
   * @brief 間引きのみ（補間なし）- デフォルト
   *
   * LaserScanの構造を維持しながら、近すぎる点を無効化（間引き）する。
   */
  LaserScan resamplePolarOnly(const LaserScan& input) const {
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
   * @brief 補間あり - XY変換→リサンプリング→元グリッド再マッピング
   *
   * XY座標でリサンプリング（補間あり）を行い、結果を元のangle_incrementグリッドに
   * 再マッピングする。歪みなし。
   */
  LaserScan resampleWithInterpolation(const LaserScan& input) const {
    // Step 1: LaserScan → XY座標（インデックス情報付き）
    std::vector<Point2D> points;
    std::vector<size_t> originalIndices;  // 元のインデックスを保持
    points.reserve(input.ranges.size());
    originalIndices.reserve(input.ranges.size());

    for (size_t i = 0; i < input.ranges.size(); ++i) {
      float range = input.ranges[i];

      if (!std::isfinite(range) || range < input.range_min || range > input.range_max) {
        continue;
      }

      double angle = input.angle_min + i * input.angle_increment;
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);
      points.emplace_back(x, y);
      originalIndices.push_back(i);
    }

    if (points.size() < 2) {
      return input;
    }

    // Step 2: XY座標でリサンプリング
    resampler_.resamplePoints(points);

    // Step 3: 元グリッドに再マッピング
    LaserScan output = input;
    // 全てをinfinityで初期化
    for (size_t i = 0; i < output.ranges.size(); ++i) {
      output.ranges[i] = std::numeric_limits<float>::infinity();
    }
    output.intensities.clear();

    // リサンプリング後の各点を元グリッドにマッピング
    for (const auto& p : points) {
      double range = std::sqrt(p.x * p.x + p.y * p.y);
      double angle = std::atan2(p.y, p.x);

      // 元グリッドでの最近傍インデックスを計算
      double index_float = (angle - input.angle_min) / input.angle_increment;
      int index = static_cast<int>(std::round(index_float));

      // 範囲チェック
      if (index >= 0 && index < static_cast<int>(output.ranges.size())) {
        // 既に値がある場合は近い方を採用
        if (!std::isfinite(output.ranges[index]) || range < output.ranges[index]) {
          output.ranges[index] = static_cast<float>(range);
        }
      }
    }

    return output;
  }
};

}  // namespace laser_scan_normalizer
