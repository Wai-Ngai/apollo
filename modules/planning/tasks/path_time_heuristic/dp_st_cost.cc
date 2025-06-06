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
#include "modules/planning/tasks/path_time_heuristic/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/speed/st_point.h"
#include "modules/planning/planning_interface_base/task_base/utils/st_gap_estimator.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      st_drivable_boundary_(st_drivable_boundary),
      init_point_(init_point),
      unit_t_(config.unit_t()),
      total_s_(total_s) {
  int index = 0;
  for (const auto& obstacle : obstacles) {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  const auto dimension_t = static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) + 1;
  boundary_cost_.resize(obstacles_.size());
  AINFO << "dimension_t: " << dimension_t << "  unit_t_ : " << unit_t_;

  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(std::vector<std::pair<double, double>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) { // s没有重合
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const {
  for (const auto& p : keep_clear_range_) {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;

  if (FLAGS_use_st_drivable_boundary) {   // false 
    // TODO(Jiancheng): move to configs
    static constexpr double boundary_resolution = 0.1;
    int index = static_cast<int>(t / boundary_resolution);
    const double lower_bound = st_drivable_boundary_.st_boundary(index).s_lower();
    const double upper_bound = st_drivable_boundary_.st_boundary(index).s_upper();

    if (s > upper_bound || s < lower_bound) {
      return kInf;
    }
  }

  // 遍历每个障碍物，计算t时刻障碍物st边界框的上界和下界，根据无人车的位置(t,s)与边界框是否重合，计算障碍物代价
  for (const auto* obstacle : obstacles_) {
    // Not applying obstacle approaching cost to virtual obstacle like created stop fences
    if (obstacle->IsVirtual()) {
      continue;
    }

    // Stop obstacles are assumed to have a safety margin when mapping them out,
    // so repelling force in dp st is not needed as it is designed to have adc
    // stop right at the stop distance we design in prior mapping process
    if (obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }

    auto boundary = obstacle->path_st_boundary();

    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {  // 纵向决策的最远距离 200
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (boundary.IsPointInBoundary(st_graph_point.point())) {
      return kInf;
    }

    // 情况4：需要减速避让或加速超过的障碍物。计算障碍物在t时刻的上界和下界位置，即上下界的累积距离s
    double s_upper = 0.0;
    double s_lower = 0.0;

    // 为了避免其他节点(t,s)再一次计算t时刻的障碍物上下界，利用缓存加速计算。GetBoundarySRange函数可以用来计算t时刻障碍物上界和下界累积距离s，并缓存
    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) { // 还没有计算过
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] = std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first; // 之前计算过，直接取值
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }

    // t时刻, 无人车在障碍物后方
    if (s < s_lower) {
      const double follow_distance_s = config_.safe_distance(); // 0.2
      AINFO << "follow_distance_s : " << follow_distance_s;
      if (s + follow_distance_s < s_lower) {                    // 如果障碍物和无人车在t时刻距离大于安全距离，距离比较远，cost=0
        continue;
      } else {                                                  // 否则距离小于安全距离，计算cost。obstacle_weight：1.0，default_obstacle_cost：1000，
        auto s_diff = follow_distance_s - s_lower + s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() * s_diff * s_diff;
      }
    // t时刻, 无人车在障碍物前方
    } else if (s > s_upper) {
      const double overtake_distance_s = StGapEstimator::EstimateSafeOvertakingGap(); // 20
      if (s > s_upper + overtake_distance_s) {  // or calculated from velocity
        continue;                               // 如果障碍物和无人车在t时刻距离大于安全距离，距离比较远，cost=0
      } else {                                  // 否则距离小于安全距离，计算cost。obstacle_weight：1.0，default_obstacle_cost：1000，
        auto s_diff = overtake_distance_s + s_upper - s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() * s_diff * s_diff;
      }
    }
  }
  return cost * unit_t_;  // unit_t_ = 1
}

double DpStCost::GetSpatialPotentialCost(const StGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();  // 距离终点惩罚 100 or 100000(变道)
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
                                      (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_; //计算时间段[t-1,t]内的平均速度
  if (speed < 0) { // 倒车？速度代价无穷大
    return kInf;
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                                                 ->GetConfig().vehicle_param()
                                                                 .max_abs_speed_when_stopped();
  AINFO << "max_adc_stop_speed : "<< max_adc_stop_speed; 
  // 如果速度接近停车，并且在禁停区内 max_stop_speed = 0.2  
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range  在KeepClear区域低速惩罚 10 * * 1000
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ * config_.default_speed_cost();
  }

  // 计算当前速度和限速的差值比，大于0，超速；小于0，未超速
  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() * (det_speed * det_speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() * -det_speed * unit_t_;
  }

  if (config_.enable_dp_reference_speed()) {
    double diff_speed = speed - cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() * fabs(diff_speed) * unit_t_;  // 10 * 1000 *  Δv * 
  }

  return cost;
}

double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;

  // 将给定的加速度 accel 标准化并加上一个偏移量 kShift 来计算得到。这样做可以确保不同的 accel 值映射到 accel_cost_ 中不同的索引位置。
  static constexpr double kEpsilon = 0.1; // 表示对加速度的精度要求
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();          // 3.0  m/s^2
    double max_dec = config_.max_deceleration();          // -4.0 m/s^2
    double accel_penalty = config_.accel_penalty();       // 1.0
    double decel_penalty = config_.decel_penalty();       // 1.0

    if (accel > 0.0) {          // 计算加速度正情况下cost
      cost = accel_penalty * accel_sq;
    } else {                    // 计算加速度负情况下cost
      cost = decel_penalty * accel_sq;
    }
    
    // 总体cost
    cost += accel_sq * decel_penalty * decel_penalty / (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty / (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key); // 该加速度之前计算过，直接索引
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  // 利用3个节点的累积距离s1，s2，s3来计算加速度   
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) {
  // 利用2个节点的累积距离s1，s2来计算加速度
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);  // 原理同acc的计算
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
