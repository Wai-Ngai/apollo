/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;

bool PathBoundsDeciderUtil::InitPathBoundary(const ReferenceLineInfo& reference_line_info,
                                             PathBoundary* const path_bound, 
                                             SLState init_sl_state) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();
  path_bound->set_delta_s(FLAGS_path_bounds_decider_resolution); // 0.5

  const auto& vehicle_config = common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_front_to_center = vehicle_config.vehicle_param().front_edge_to_center();

  // 从规划起点开始, 每隔0.5m取一个点s, 初始化l为±最大. 规划长度与巡航速度有关v*8, 至少100m, 但不能超过参考线长度
  for (double curr_s = init_sl_state.first[0];
       curr_s < std::fmin(init_sl_state.first[0] + std::fmax(FLAGS_path_bounds_horizon,          // 100
                                                             reference_line_info.GetCruiseSpeed() * FLAGS_trajectory_time_length),  // 8s
                          reference_line.Length() - ego_front_to_center);
       curr_s += FLAGS_path_bounds_decider_resolution) {

    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }
  AINFO << " start_s : " << init_sl_state.first[0];
  AINFO << " end_s : " << path_bound->back().s; 
  AINFO << " GetCruiseSpeed : " << reference_line_info.GetCruiseSpeed();

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

void PathBoundsDeciderUtil::GetStartPoint(common::TrajectoryPoint planning_start_point,
                                          const ReferenceLine& reference_line, SLState* init_sl_state) {
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point = InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  *init_sl_state = reference_line.ToFrenetFrame(planning_start_point);
}

double PathBoundsDeciderUtil::GetADCLaneWidth(const ReferenceLine& reference_line, const double adc_s) {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  
  if (!reference_line.GetLaneWidth(adc_s, &lane_left_width,
                                   &lane_right_width)) {
    constexpr double kDefaultLaneWidth = 5.0;
    AWARN << "Failed to get lane width at planning start point.";
    return kDefaultLaneWidth;
  } else {
    return lane_left_width + lane_right_width;
  }
}

bool PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(double left_bound, double right_bound, 
                                                         BoundType left_type,BoundType right_type, 
                                                         std::string left_id, std::string right_id,
                                                         PathBoundPoint* const bound_point) {
  if (!UpdateLeftPathBoundaryWithBuffer(left_bound, left_type, left_id,
                                        bound_point)) {
    return false;
  }
  if (!UpdateRightPathBoundaryWithBuffer(right_bound, right_type, right_id,
                                         bound_point)) {
    return false;
  }
  return true;
}

bool PathBoundsDeciderUtil::UpdateLeftPathBoundaryWithBuffer(double left_bound, BoundType left_type, 
                                                             std::string left_id, PathBoundPoint* const bound_point) {
  left_bound = left_bound - GetBufferBetweenADCCenterAndEdge(); // 减去自车半宽

  PathBoundPoint new_point = *bound_point;
  if (new_point.l_upper.l > left_bound) {
    new_point.l_upper.l = left_bound;     // 更新上边界
    new_point.l_upper.type = left_type;   // apollo::planning::LANE
    new_point.l_upper.id = left_id;       // ''
  }

  // Check if ADC is blocked. If blocked, don't update anything, return false.
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at" << new_point.l_lower.l << " "
           << new_point.l_upper.l;
    return false;
  }

  // Otherwise, update path_boundaries and center_line; then return true.
  *bound_point = new_point;
  return true;
}

bool PathBoundsDeciderUtil::UpdateRightPathBoundaryWithBuffer(double right_bound, BoundType right_type, 
                                                              std::string right_id, PathBoundPoint* const bound_point) {
  right_bound = right_bound + GetBufferBetweenADCCenterAndEdge();

  PathBoundPoint new_point = *bound_point;
  if (new_point.l_lower.l < right_bound) {
    new_point.l_lower.l = right_bound;
    new_point.l_lower.type = right_type;
    new_point.l_lower.id = right_id;
  }

  // Check if ADC is blocked. If blocked, don't update anything, return false.
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at";
    return false;
  }

  // Otherwise, update path_boundaries and center_line; then return true.
  *bound_point = new_point;
  return true;
}

void PathBoundsDeciderUtil::TrimPathBounds(const int path_blocked_idx, 
                                           PathBoundary* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }

    // 计算被遮挡的长度，把挡住后的那段路去掉
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

std::string PathBoundsDeciderUtil::FindFarthestBlockObstaclesId(const std::unordered_map<std::string, double>& obs_id_to_start_s) {
  std::string nearest_obstcles_id = "";
  double max_start_s = std::numeric_limits<double>::lowest();

  for (auto obs : obs_id_to_start_s) {
    if (obs.second > max_start_s) {
      nearest_obstcles_id = obs.first;
      max_start_s = obs.second;
    }
  }
  return nearest_obstcles_id;
}

bool CompareLeftBound(const std::pair<std::string, double>& lhs,
                      const std::pair<std::string, double>& rhs) {
  if (lhs.first == rhs.first) {
    return false;
  }
  return lhs.second < rhs.second;
}

bool CompareRightBound(const std::pair<std::string, double>& lhs,
                       const std::pair<std::string, double>& rhs) {
  if (lhs.first == rhs.first) {
    return false;
  }
  return lhs.second > rhs.second;
}

bool PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(const ReferenceLineInfo& reference_line_info, 
                                                           const SLState& init_sl_state,
                                                           PathBoundary* const path_boundaries,
                                                           std::string* const blocking_obstacle_id,
                                                           double* const narrowest_width) {
  // Preprocessing.
  auto indexed_obstacles = reference_line_info.path_decision().obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles, init_sl_state);
  AINFO << "There are " << sorted_obstacles.size() << " obstacles.";

  double center_line = init_sl_state.second[0];
  AINFO << "init l " << init_sl_state.second[0];

  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<std::pair<std::string, double>, decltype(CompareRightBound)*> right_bounds(CompareRightBound); // 右边界按 second 值降序排列
  std::multiset<std::pair<std::string, double>, decltype(CompareLeftBound)*> left_bounds(CompareLeftBound);    // 左边界按 second 值升序排列
  right_bounds.insert(std::make_pair("", std::numeric_limits<double>::lowest()));
  left_bounds.insert(std::make_pair("", std::numeric_limits<double>::max()));

  // Maps obstacle ID's to the decided ADC pass direction, if ADC should pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;
  // Maps obstacle ID's to start s on this obstacle
  std::unordered_map<std::string, double> obs_id_to_start_s;
  
  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = (*path_boundaries)[i].s;
    AINFO << "curr_s  " << curr_s;

    // Check and see if there is any obstacle change;
    // 从第一个 s 小于 curr_obstacle_s的点开始更新该点的边界
    if (obs_idx < sorted_obstacles.size() && std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {

      while (obs_idx < sorted_obstacles.size() && std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        // 获取障碍物的边界
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);

        AINFO << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
               << "] curr_obstacle_l_min[" << curr_obstacle_l_min
               << "] curr_obstacle_l_max[" << curr_obstacle_l_max
               << "] center_line[" << center_line << "]";

        // 障碍物起点，判断障碍物在道路中线的左边还是右边，决定自车是左绕还是右绕，从而决定道路边界
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.

          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(std::make_pair(curr_obstacle_id, curr_obstacle_l_max));

            AINFO << curr_obstacle_id << " left nudge";
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(std::make_pair(curr_obstacle_id, curr_obstacle_l_min));

            AINFO << curr_obstacle_id << " right nudge";
          }
          obs_id_to_start_s[curr_obstacle_id] = curr_obstacle_s;
        } else {
          // 障碍物终点，向左绕，从右边边界集合去除；向右绕，从左边边界集合去除
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(std::make_pair(curr_obstacle_id, curr_obstacle_l_max)));
          } else {
            left_bounds.erase(left_bounds.find(std::make_pair(curr_obstacle_id, curr_obstacle_l_min)));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
          obs_id_to_start_s.erase(curr_obstacle_id);
        }
        // AINFO << " left_bound : " << left_bounds.begin()->second;
        // AINFO << "right_bound : " << right_bounds.begin()->second;

        // Update the bounds and center_line.
        if (!UpdateLeftPathBoundaryWithBuffer(left_bounds.begin()->second, 
                                              BoundType::OBSTACLE,
                                              left_bounds.begin()->first, &(*path_boundaries)[i])) {
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_start_s.empty()) {
            *blocking_obstacle_id = FindFarthestBlockObstaclesId(obs_id_to_start_s);
          }
          break;
        }
        if (!UpdateRightPathBoundaryWithBuffer(right_bounds.begin()->second, 
                                               BoundType::OBSTACLE,
                                               right_bounds.begin()->first, &(*path_boundaries)[i])) {
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_start_s.empty()) {
            *blocking_obstacle_id = FindFarthestBlockObstaclesId(obs_id_to_start_s);
          }
          break;
        }

        ++obs_idx;
      }
    } else {
      // 跟新完第一个点后，后面都走else逻辑，将边界全部更新成障碍物边界，直到障碍物最后一个点。
      // If no obstacle change, update the bounds and center_line.
      if (!UpdateLeftPathBoundaryWithBuffer(left_bounds.begin()->second, 
                                            BoundType::OBSTACLE,
                                            left_bounds.begin()->first, &(*path_boundaries)[i])) {
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_start_s.empty()) {
          *blocking_obstacle_id = FindFarthestBlockObstaclesId(obs_id_to_start_s);
        }
        break;
      }
      if (!UpdateRightPathBoundaryWithBuffer(right_bounds.begin()->second, 
                                             BoundType::OBSTACLE,
                                             right_bounds.begin()->first, &(*path_boundaries)[i])) {
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_start_s.empty()) {
          *blocking_obstacle_id = FindFarthestBlockObstaclesId(obs_id_to_start_s);
        }
        break;
      }
    }
    AINFO << " left_bound : " << (*path_boundaries)[i].l_upper.l;
    AINFO << "right_bound : " << (*path_boundaries)[i].l_lower.l;
    
    center_line = ((*path_boundaries)[i].l_lower.l + (*path_boundaries)[i].l_upper.l) / 2.0;
    // AINFO << "center_line : " << center_line;

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }
  AINFO << "blocking_obstacle_id" << *blocking_obstacle_id << ","
        << path_blocked_idx;
  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<ObstacleEdge> PathBoundsDeciderUtil::SortObstaclesForSweepLine(const IndexedList<std::string, Obstacle>& indexed_obstacles,
                                                                           const SLState& init_sl_state) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC. 只考自车前面的障碍物
    if (obstacle->PerceptionSLBoundary().end_s() < init_sl_state.first[0]) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();

    // 将障碍物起点和终点分别放入容器，障碍物四周膨胀一个buffer
    sorted_obstacles.emplace_back(1, 
                                  obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer, // 3.0
                                  obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,       // 0.4
                                  obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, 
                                  obstacle->Id());

    sorted_obstacles.emplace_back(0, 
                                  obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,     // 2.0
                                  obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
                                  obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, 
                                  obstacle->Id());
    AINFO << " ID [" << obstacle->Id() << "]  : " 
          << obstacle_sl.start_s() << " - " << obstacle_sl.end_s() << "  "
          << obstacle_sl.start_l() << " - " << obstacle_sl.end_l() << "  ";
  }

  // Sort. 根据s，从小到大进行排序
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

double PathBoundsDeciderUtil::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width = VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many factors such as: ADC length, possible turning angle, speed, etc.
  static constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() ||
      obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }

  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

bool PathBoundsDeciderUtil::ComputeSLBoundaryIntersection(const SLBoundary& sl_boundary, 
                                                          const double s, double* ptr_l_min,
                                                          double* ptr_l_max) {
  *ptr_l_min = std::numeric_limits<double>::max();
  *ptr_l_max = -std::numeric_limits<double>::max();

  // invalid polygon
  if (sl_boundary.boundary_point_size() < 3) {
    return false;
  }

  bool has_intersection = false;
  for (auto i = 0; i < sl_boundary.boundary_point_size(); ++i) {
    auto j = (i + 1) % sl_boundary.boundary_point_size();
    const auto& p0 = sl_boundary.boundary_point(i);
    const auto& p1 = sl_boundary.boundary_point(j);

    if (common::util::WithinBound<double>(std::fmin(p0.s(), p1.s()),
                                          std::fmax(p0.s(), p1.s()), s)) {
      has_intersection = true;
      auto l = common::math::lerp<double>(p0.l(), p0.s(), p1.l(), p1.s(), s);
      if (l < *ptr_l_min) {
        *ptr_l_min = l;
      }
      if (l > *ptr_l_max) {
        *ptr_l_max = l;
      }
    }
  }
  return has_intersection;
}

common::TrajectoryPoint PathBoundsDeciderUtil::InferFrontAxeCenterFromRearAxeCenter(const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance = VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;

  ret.mutable_path_point()->set_x(traj_point.path_point().x() +
                                  front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(traj_point.path_point().y() +
                                  front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

bool PathBoundsDeciderUtil::GetBoundaryFromSelfLane(const ReferenceLineInfo& reference_line_info, 
                                                    const SLState& init_sl_state,
                                                    PathBoundary* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  // 计算目标车道宽度
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double adc_lane_width = GetADCLaneWidth(reference_line, init_sl_state.first[0]);
  AINFO << "adc_lane_width : " << adc_lane_width;

  // Go through every point, update the boundary based on lane info and ADC's position.
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;

  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    // AINFO << "curr_s : " << curr_s;

    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }
    // AINFO << "past_lane_left_width  : " << past_lane_left_width;
    // AINFO << "past_lane_right_width : " << past_lane_right_width;


    // 3. Calculate the proper boundary based on lane-width, ADC's position, and ADC's velocity.
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    curr_left_bound = curr_lane_left_width - offset_to_map;
    curr_right_bound = -curr_lane_right_width - offset_to_map;

    // AINFO << "offset_to_map    : " << offset_to_map;
    // AINFO << "curr_left_bound  : " << curr_left_bound;
    // AINFO << "curr_right_bound : " << curr_right_bound;

    // 4. Update the boundary. 减去自车半宽，表示自车中心可以变化的l范围
    if (!UpdatePathBoundaryWithBuffer(curr_left_bound, curr_right_bound,
                                      BoundType::LANE, BoundType::LANE, "", "",
                                      &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) { // 路被第i个点挡住了，直接返回
      break;
    }
    AINFO << "l_upper          : " << path_bound->at(i).l_upper.l;
    AINFO << "l_lower          : " << path_bound->at(i).l_lower.l;
  }

  // 把挡住后的那段路去掉
  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);

  return true;
}

bool PathBoundsDeciderUtil::ExtendBoundaryByADC(const ReferenceLineInfo& reference_line_info, 
                                                const SLState& init_sl_state, // (s ,s' ,s''), (l, l', l'')
                                                const double extend_buffer,   // 0.5(LC) 0.2(LK)
                                                PathBoundary* const path_bound) {
  // 根据侧向速度，扩展变道那侧的边界
  double adc_l_to_lane_center = init_sl_state.second[0];  // 向左变道，为负，变道刚过线，就变道完成，此时 l = -1.61左右
  static constexpr double kMaxLateralAccelerations = 1.5; 

  double ADC_speed_buffer = (init_sl_state.second[1] > 0 ? 1.0 : -1.0) *
                            init_sl_state.second[1] * init_sl_state.second[1] /
                            kMaxLateralAccelerations / 2.0; // 向左变道，为正
  double adc_half_width = VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // ADC_speed_buffer过小，所以在left_bound_adc上额外添加了半个车身，留多一些空间
  double left_bound_adc = std::fmax(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) +
                          adc_half_width + extend_buffer;   // 随着变道，左右边界逐渐收缩
  double right_bound_adc = std::fmin(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) -
                           adc_half_width - extend_buffer;
  
  AINFO << " adc_l_to_lane_center " << adc_l_to_lane_center;
  AINFO << " init_sl_state.second[1] " << init_sl_state.second[1];
  AINFO << " adc_half_width   " << adc_half_width;
  AINFO << " ADC_speed_buffer " << ADC_speed_buffer;
  AINFO << " left_bound_adc   " << left_bound_adc;
  AINFO << " right_bound_adc  " << right_bound_adc;

  // 根据道路宽度，扩展边界
  static constexpr double kEpsilon = 0.05;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double road_left_width = std::fabs(left_bound_adc) + kEpsilon;
    double road_right_width = std::fabs(right_bound_adc) + kEpsilon;

    // 计算道路边界，以目标车道中心线为s轴，均为正
    reference_line_info.reference_line().GetRoadWidth((*path_bound)[i].s, 
                                                      &road_left_width, 
                                                      &road_right_width);
    // AINFO << " road_left_width  " << road_left_width;
    // AINFO << " road_right_width " << road_right_width;

    // 减去半车宽，自车中心实际可运动的道路边界，左正右负
    double left_bound_road = road_left_width - adc_half_width;
    double right_bound_road = -road_right_width + adc_half_width;
    // AINFO << " left_bound_road  " << left_bound_road;
    // AINFO << " right_bound_road " << right_bound_road;

    if (left_bound_adc > (*path_bound)[i].l_upper.l) {  // 向右变道，更新左边界（原左边界是车道边界）为道路边界，变道过程中，左边界逐渐收缩
      (*path_bound)[i].l_upper.l = std::max(std::min(left_bound_adc, left_bound_road),
                                            (*path_bound)[i].l_upper.l);
      (*path_bound)[i].l_upper.type = BoundType::ADC;
      (*path_bound)[i].l_upper.id = "adc";
      
      AINFO << " l_upper          " << (*path_bound)[i].l_upper.l;
    }

    if (right_bound_adc < (*path_bound)[i].l_lower.l) { // 向左变道，更新右边界（原右边界是车道边界）为道路边界，变道过程中，右边界逐渐收缩
      (*path_bound)[i].l_lower.l = std::min(std::max(right_bound_adc, right_bound_road),
                                            (*path_bound)[i].l_lower.l);
      (*path_bound)[i].l_lower.type = BoundType::ADC;
      (*path_bound)[i].l_lower.id = "adc";

      AINFO << " l_lower          " << (*path_bound)[i].l_lower.l;
    }
  }
  return true;
}

void PathBoundsDeciderUtil::ConvertBoundarySAxisFromLaneCenterToRefLine(const ReferenceLineInfo& reference_line_info,
                                                                        PathBoundary* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double refline_offset_to_lane_center = 0.0;

    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);

    (*path_bound)[i].l_lower.l -= refline_offset_to_lane_center;
    (*path_bound)[i].l_upper.l -= refline_offset_to_lane_center;
  }
}

int PathBoundsDeciderUtil::IsPointWithinPathBound(const ReferenceLineInfo& reference_line_info, 
                                                  const double x, const double y, 
                                                  const PathBound& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);

  if (point_sl.s() > path_bound.back().s ||
      point_sl.s() <
          path_bound.front().s - FLAGS_path_bounds_decider_resolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }

  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         path_bound[idx_after].s < point_sl.s()) {
    ++idx_after;
  }

  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << path_bound[idx_after].l_lower.l << ", "
         << path_bound[idx_after].l_upper.l << "].";
  ADEBUG << "The point is at: " << point_sl.l();

  int idx_before = idx_after - 1;
  if (path_bound[idx_before].l_lower.l <= point_sl.l() &&
      path_bound[idx_before].l_upper.l >= point_sl.l() &&
      path_bound[idx_after].l_lower.l <= point_sl.l() &&
      path_bound[idx_after].l_upper.l >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

}  // namespace planning
}  // namespace apollo
