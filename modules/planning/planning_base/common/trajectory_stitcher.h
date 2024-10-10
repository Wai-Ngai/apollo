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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(const double x_diff, const double y_diff, const double theta_diff,
                                               PublishableTrajectory* prev_trajectory);

  /**
   * @brief 如果触发replan的条件就返回replan的轨迹(里面就一个点车辆当前状态轨迹点/或运动学推导0.1s后对应车辆状态的轨迹点)
   *        如果没触发replan就进行轨迹拼接，在上一次的规划轨迹中从车辆当前状态时间最近点/距离最近点(取较小的那个)往前截取20个点开始，
   *        到(当前时间戳+规划周期0.1s在上一段规划轨迹对应的点)，这一段轨迹作为待拼接轨迹，最后一个点作为下一段规划轨迹的纵向起点s=0的点，时间戳以当前时间戳作为下段规划轨迹时间起点
   * 
   * @param vehicle_chassis 
   * @param vehicle_state 
   * @param current_timestamp 
   * @param planning_cycle_time 
   * @param preserved_points_num  保留的点的数量
   * @param replan_by_offset      是否因为offset而replan
   * @param prev_trajectory 
   * @param replan_reason 
   * @return std::vector<common::TrajectoryPoint> 
   */
  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(const canbus::Chassis& vehicle_chassis,
                                                                         const common::VehicleState& vehicle_state,
                                                                         const double current_timestamp, const double planning_cycle_time,
                                                                         const size_t preserved_points_num, const bool replan_by_offset,
                                                                         const PublishableTrajectory* prev_trajectory, std::string* replan_reason);

  /**
   * @brief 如果车速加速度够小，以当前状态为0.1s后规划轨迹点；若车速和加速度不够小就以运动学模型预测0.1s后的车辆状态作为规划轨迹的0.1s处的点
   * 
   * @param planning_cycle_time 
   * @param vehicle_state 
   * @return std::vector<common::TrajectoryPoint> 
   */
  static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(const double planning_cycle_time,
                                                                               const common::VehicleState& vehicle_state);

 private:
 /**
  * @brief 计算位置投影 其实就是将一个x,y坐标投影到轨迹点p上转化为sd坐标，s代表纵向，d代表横向
  * 
  * @param x 
  * @param y 
  * @param matched_trajectory_point 
  * @return std::pair<double, double> 
  */
  static std::pair<double, double> ComputePositionProjection(const double x, const double y,
                                                             const common::TrajectoryPoint& matched_trajectory_point);

  /**
   * @brief 把车辆状态当前的s,x,y,z,theta,kappa,v,a设置到一个轨迹点，
   * 并将车辆当前状态的这个轨迹点相对时间设置为0.1s(考虑的规划执行周期)，
   * 然后返回这个轨迹点(s,x,y,z,theta,kappa,v,a,relative_time=0.1s)
   * 
   * @param planning_cycle_time 
   * @param vehicle_state 
   * @return common::TrajectoryPoint 
   */
  static common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(const double planning_cycle_time,
                                                                        const common::VehicleState& vehicle_state);
};

}  // namespace planning
}  // namespace apollo
