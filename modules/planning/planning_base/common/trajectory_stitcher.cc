/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/planning_base/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(const double planning_cycle_time, 
                                                                           const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}

std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeReinitStitchingTrajectory(const double planning_cycle_time, 
                                                                                  const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided

  // 车速/加速度若很小时，直接将车辆当前状态的点作为规划起点
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    //那么如果车速和加速度不够小时，根据 运动学模型 预测一个规划周期0.1s后的车辆状态
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state = VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time, predicted_vehicle_state);
  }

  // 返回的轨迹里就1个点
  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(const double x_diff, const double y_diff, const double theta_diff,
                                                          PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(const canbus::Chassis& vehicle_chassis, 
                                                                            const VehicleState& vehicle_state,
                                                                            const double current_timestamp, 
                                                                            const double planning_cycle_time,
                                                                            const size_t preserved_points_num, 
                                                                            const bool replan_by_offset,
                                                                            const PublishableTrajectory* prev_trajectory, 
                                                                            std::string* replan_reason) {
  // 1. 是否使能轨迹拼接
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 2. 上一帧是否生成轨迹
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 3. 是否处于自动驾驶模式
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 4. 上一帧是否存在轨迹点 
  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();
  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  // 计算 时间匹配点
  const double veh_rel_time = current_timestamp - prev_trajectory->header_time();
  size_t time_matched_index = prev_trajectory->QueryLowerBoundPoint(veh_rel_time);
  AINFO << "veh_rel_time : " << veh_rel_time;

  // 5. 当前时间小于上一轨迹的初始时间时
  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason = "replan for current time smaller than the previous trajectory's first time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 6. 当前时间大于上一轨迹的最终时间时
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason = "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  // 7. 该时刻无法在上一帧轨迹中找到匹配点时
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(static_cast<uint32_t>(time_matched_index));
  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  // 计算 位置匹配点
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer({vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(vehicle_state.x(), vehicle_state.y(),
                                             prev_trajectory->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index)));

  // 判断横纵向偏差
  if (replan_by_offset) {
    // 8.驻车刹车松开（停车起步）
    if (vehicle_chassis.has_parking_brake()) {
      static bool parking_brake = true;
      if (parking_brake && !vehicle_chassis.parking_brake()) {
        parking_brake = vehicle_chassis.parking_brake();
        const std::string msg = "parking brake off, ego move, replan to avoid large station error";
        AERROR << msg;
        *replan_reason = msg;
        return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
      }
      parking_brake = vehicle_chassis.parking_brake();
    }

    // 判断横纵向偏差
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    double time_diff = time_matched_point.relative_time() -
                       prev_trajectory->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index)).relative_time();
    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff
           << ", time diff: " << time_diff;

    // 横向偏差不满足条件 0.5m
    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
    }

    // 纵向偏差不满足条件 2.5m
    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
    }
    // 时间阈值 7s
    if (std::fabs(time_diff) > FLAGS_replan_time_threshold) {
      const std::string msg = absl::StrCat(
          "the difference between time matched point relative time and "
          "actual position corresponding relative time is too "
          "large. Replan is triggered. time_diff = ",
          time_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of "
           << "lat、lon and time offset is disabled";
  }

  // 计算当前时刻后△T时刻，及其在上一帧轨迹中对应的索引值
  double forward_rel_time = veh_rel_time + planning_cycle_time;
  size_t forward_time_index = prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  // 选择时间匹配索引和位置匹配索引中的较小索引作为匹配点索引
  auto matched_index = std::min(time_matched_index, position_matched_index);

  // 构建拼接轨迹，<匹配索引点前n个点，当前时刻后的T时刻所对应的匹配点>
  std::vector<TrajectoryPoint> stitching_trajectory(prev_trajectory->begin() + std::max(0, static_cast<int>(matched_index - preserved_points_num)),
                                                    prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
    }
    // 适配时间和s值
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() - current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(const double x, const double y, 
                                                                        const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(),
          y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), 
          std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
