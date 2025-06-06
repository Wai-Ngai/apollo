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
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;

// Continuous-time collision check using linear interpolation as closed-loop dynamics
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  if (FLAGS_use_st_drivable_boundary) {
    return false;
  }
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
  }
  return false;
}
}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(const StGraphData& st_graph_data, 
                                           const DpStSpeedOptimizerConfig& dp_config,
                                           const std::vector<const Obstacle*>& obstacles,
                                           const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),   // 速度边界里面赋值，ST图
      gridded_path_time_graph_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) {

  total_length_t_ = st_graph_data_.total_time_by_conf();    // 7
  unit_t_ = gridded_path_time_graph_config_.unit_t();
  total_length_s_ = st_graph_data_.path_length();
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();           // 稠密, 0.1, 0.25(变道)
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();         // 稀疏, 1.0
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s(); //      101, 21(变道)
  
  AINFO << "total_length_t_  " <<total_length_t_ << " unit_t_ " << unit_t_<< " total_length_s_ " <<total_length_s_;


  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ = std::min(std::abs(vehicle_param_.max_acceleration()),
                               std::abs(gridded_path_time_graph_config_.max_acceleration()));         // 2
  max_deceleration_ = -1.0 * std::min(std::abs(vehicle_param_.max_deceleration()),
                                      std::abs(gridded_path_time_graph_config_.max_deceleration()));  // -4, -2.5(变道)
}

Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {
  static constexpr double kBounadryEpsilon = 1e-2;
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // If init point in collision with obstacle, return speed fallback
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }

  // 初始化代价表，每个栅格点的坐标为其在st图中的坐标，每个栅格点的cost都初始化为无穷大。
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 初始化限速表，初始化每个s位置的限速
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 递推计算每个栅格点的代价
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 从终点回溯获得完整st曲线
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;

  double sparse_length_s = total_length_s_ -
                           static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon()
                        ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
                        : 0;
  dense_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon()
                       ? dense_dimension_s_
                       : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) + 1;
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;
  AINFO << "dimension_t_  " << dimension_t_ << " sparse_length_s " << sparse_length_s 
        << " dense_dimension_s_ " << dense_dimension_s_ << " dimension_s_" << dimension_s_;

  PrintCurves debug;
  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 定义dp代价表，注意这个表：常规ST图顺时针旋转90°，一共t行，每行s列 , total_cost_默认无穷大
  cost_table_ = std::vector<std::vector<StGraphPoint>>(dimension_t_, 
                                                       std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {          // T
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;

    // 稠密
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {  // S
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
      debug.AddPoint("dp_node_points", curr_t, curr_s);
    }

    // 稀疏
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ + sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size(); ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
      debug.AddPoint("dp_node_points", curr_t, curr_s);
    }
  }

  // 获取第一行的s
  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();

  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] = speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {     // 循环遍历cost_table_的行数, T。
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;  // 最大s的index

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      std::vector<std::future<void>> results;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);

        if (gridded_path_time_graph_config_.enable_multi_thread_in_dp_st_graph()) {
          results.push_back(cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
           // 计算规划起点到当前点cost_table_[c][r]的最小代价
          CalculateCostAt(msg);
        }
      }
      if (gridded_path_time_graph_config_.enable_multi_thread_in_dp_st_graph()) {
        for (auto& result : results) {
          result.get();
        }
      }
    }

    // 为了减少计算量，根据当前节点的速度和时间差，计算下一行可以达到的列范围。这样下一行中并不是所有节点都会计算cost
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);           // 获取该列的范围
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)

  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else { // 如果当前节点的前一个点存在，计算前一个点到当前节点的斜率，即速度：v = △s / △t
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;

  // 根据当前节点的速度，计算最远可以到的s， s = v0*t + 0.5*a*t^2 + s0
  const double s_upper_bound = v0 * unit_t_ + acc_coeff * max_acceleration_ * t_squared + point.point().s();
  const auto next_highest_itr = std::lower_bound(spatial_distance_by_index_.begin(),
                                                 spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row = std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound = std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) + point.point().s();
  const auto next_lowest_itr = std::lower_bound(spatial_distance_by_index_.begin(),
                                                spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row = std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

// 计算规划起点到当前点cost_table_[c][r]的最小代价
void GriddedPathTimeGraph::CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;    // T
  const uint32_t r = msg->r;    // S
  auto& cost_cr = cost_table_[c][r];
  AINFO << "t: " << cost_cr.point().t() <<"  s: " << cost_cr.point().s();

  // 1.障碍物代价
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return; // 障碍物代价inf，后面就不用算了
  }

  // 2.距离终点代价
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0];
  if (c == 0) { // t=0，第一行不计算
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  if (c == 1) { // t=1，计算规划起点到第2行每个点的代价
    // 加速度超过自车正常区间的点，直接返回
    const double acc = 2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_; // s = vo * t + 1/2 * a * t^2
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;
    }

    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    // 检查规划起点和当前点的连线与ST图是否有重叠
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), 
                                cost_cr, cost_init)) {
      return;
    }
    AINFO << "cost_init.total_cost() : " << cost_init.total_cost() ;

    // 计算总的代价
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + 
                         cost_cr.spatial_potential_cost() +
                         cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));  // 计算第2列的边代价
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;
  }

  // 计算前一行最低点s
  static constexpr double kSpeedRangeBuffer = 0.20;
  const double pre_lowest_s = cost_cr.point().s() -
                              FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  const auto pre_lowest_itr = std::lower_bound(spatial_distance_by_index_.begin(),
                                               spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1; // s
  const auto& pre_col = cost_table_[c - 1];  // 
  double curr_speed_limit = speed_limit;

  // 计算第3行的代价
  if (c == 2) {
    for (uint32_t i = 0; i < r_pre_size; ++i) { // 遍历前一行的有效范围内的所有节点, 计算该节点到本节点的代价
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;  // 前一个节点如果代价Inf 或者节点为空，就不再计算前一个节点到当前列节点的代价
      }

      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed data in ST point.
      // Current v:  curr_v = (point.s - pre_point.s) / unit_t
      // Previous v: pre_v = (pre_point.s - prepre_point.s) / unit_t
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      //                              = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

      const double curr_a = 2 * ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
                                  pre_col[r_pre].GetOptimalSpeed()) / unit_t_;    // s = vo * t + 1/2 * a * t^2
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), 
                                  cost_cr, pre_col[r_pre])) {
        continue;
      }

      curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit, cruise_speed);  // 计算第3列的边代价
  
      AINFO << "cost_cr.total_cost() : " << cost_cr.total_cost() ;

      // 对当前节点(t,s)选择一个具有最小cost的父节点
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);   // v0 + a * t
      }
    }
    return;
  }

  // 计算第4行及以后的代价
  for (uint32_t i = 0; i < r_pre_size; ++i) { // 遍历前一行的有效范围内的所有节点, 计算该节点到本节点的代价
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }

    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    //                              = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

    const double curr_a = 2 * ((cost_cr.point().s() - pre_col[r_pre].point().s()) /
                          unit_t_ - pre_col[r_pre].GetOptimalSpeed()) / unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }
    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();

    curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

    double cost = cost_cr.obstacle_cost() + 
                  cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed); // 计算第4行的边代价

    // 选择前一行中与当前节点(t,s)连接的代价cost最小的节点
    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
    }
  }
}

double GriddedPathTimeGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second, 
                                               const STPoint& third, const STPoint& forth, 
                                               const double speed_limit, const double cruise_speed) {
  // 速度代价 + 加速度代价 + jerk代价  
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(const uint32_t row, 
                                                           const double speed_limit, 
                                                           const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();

  // 速度代价 + 加速度代价 + jerk代价  
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point, curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point, curr_point);
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row, const uint32_t pre_row, 
                                                          const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  
  // 速度代价 + 加速度代价 + jerk代价  
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}


Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  // Step 1 : 计算规划终点
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  PrintPoints debug("dp_node_edge");
  for (const auto& points_vec : cost_table_) {
    for (const auto& pt : points_vec) {
      debug.AddPoint(pt.point().t(), pt.point().s());
    }
  }
  // for debug plot
  // debug.PrintToLog();

  // 寻找最上一行(s=S)和最右一列(t=T)中最小的cost对应的节点，作为规划终点
  for (const StGraphPoint& cur_point : cost_table_.back()) {  // 最右一列(t=T)
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  for (const auto& row : cost_table_) {                      // 每一个t，也就是每一列
    const StGraphPoint& cur_point = row.back();              // 每一列的最上一行(s=S)
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Step 2 : 从规划终点开始回溯，找到最小cost的规划路径
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  PrintPoints debug_res("dp_result");
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();

    SpeedPoint speed_point;
    debug_res.AddPoint(cur_point->point().t(), cur_point->point().s());
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);

    cur_point = cur_point->pre_point();
  }
  //  for debug plot
  //   debug_res.PrintToLog();
  std::reverse(speed_profile.begin(), speed_profile.end());  // 颠倒容器中元素的顺序

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();  // 返回的是 double 类型能够表示的最小正数
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  AINFO << "front: " << speed_profile.front().t() << " " << speed_profile.front().s();


  // Step 3 : 计算速度 v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);  // 斜率 
    speed_profile[i].set_v(v);
    AINFO << "v: " << v;
  }

  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}


}  // namespace planning
}  // namespace apollo
