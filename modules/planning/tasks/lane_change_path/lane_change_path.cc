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

#include "modules/planning/tasks/lane_change_path/lane_change_path.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool LaneChangePath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneChangePathConfig>(&config_);
}

apollo::common::Status LaneChangePath::Process(Frame* frame, 
                                               ReferenceLineInfo* reference_line_info) {
  // 根据车辆状态、参考线数量以及上一帧车辆是否处于换道状态判断是否换道。
  UpdateLaneChangeStatus();

  // 非变道场景或路径重用，不执行变道路径规划。
  const auto& status = injector_->planning_context()->mutable_planning_status()
                                ->mutable_change_lane()->status();

  AINFO << " Current : " << injector_->planning_context()->mutable_planning_status()
                                     ->mutable_change_lane()->DebugString();

  if (!reference_line_info->IsChangeLanePath() ||
      reference_line_info->path_reusable()) {
    AINFO << "Skip this time" << reference_line_info->IsChangeLanePath()
          << "path reusable" << reference_line_info->path_reusable();
    return Status::OK();
  }
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) {
    ADEBUG << injector_->planning_context()->mutable_planning_status()
                       ->mutable_change_lane()->DebugString();
    AINFO << " Not satisfy lane change conditions !  ";
    return Status(ErrorCode::PLANNING_ERROR, "Not satisfy lane change conditions");
  }

  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  // 规划起点cartesian转换为frenet坐标。如果使用前轴中心作为规划起点，将规划起点平移。（代码上看，默认后轴作为规划起点）
  GetStartPointSLState();

  // 通过对周围车道、车辆位置和静态障碍物的分析，来决定车辆行驶的路径边界
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status(ErrorCode::PLANNING_ERROR, "lane change path bounds failed");
  }

  // 优化路径
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status(ErrorCode::PLANNING_ERROR, "lane change path optimize failed");
  }

  // 评估候选路径数据，并选择一个最终路径
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    return Status(ErrorCode::PLANNING_ERROR, "No valid lane change path");
  }

  return Status::OK();
}

bool LaneChangePath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  double path_narrowest_width = 0;

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, 
                                               init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, 
                                                      init_sl_state_, 
                                                      &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
  // 3. 根据自车侧向速度以及道路宽度 对路径边界进行拓展
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(*reference_line_info_, 
                                                  init_sl_state_, 
                                                  config_.extend_adc_buffer(), // 0.5
                                                  &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on adc.";
    return false;
  }

  // 4. Remove the S-length of target lane out of the path-bound.
  // 根据障碍物是否影响变道，对换道点前的路径边界进行更新
  GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

  PathBound temp_path_bound = path_bound;
  std::string blocking_obstacle_id;
  // 5. 根据静态障碍物在参考线上的投影再次更新路径边界
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(*reference_line_info_, init_sl_state_, 
                                                             &path_bound, &blocking_obstacle_id, 
                                                             &path_narrowest_width)) {
    AERROR << "Failed to decide fine tune the boundaries after "
              "taking into consideration all static obstacles.";
    return false;
  }

  // 6. Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }
  path_bound.set_label("regular/lane_change");
  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

bool LaneChangePath::OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                                  std::vector<PathData>* candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  AINFO << "path_boundaries.size() :  " << path_boundaries.size();

  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    AINFO << "path_boundary.boundary().size() :  " << path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }

    std::vector<double> opt_l, opt_dl, opt_ddl;

    // 曲率约束计算, 根据frenet坐标转换公式，实际规划路径的曲率和l的二阶导有关系。
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);

    // 曲率变化率约束, 也就是要规划出满足方向盘转角范围内。
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);

    // 进行路径优化
    bool res_opt = PathOptimizerUtil::OptimizePath(init_sl_state_, end_state, 
                                                   ref_l, weight_ref_l, path_boundary,
                                                   ddl_bounds, jerk_bound, config, 
                                                   &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      // 转换为分段jerk路径
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, 
                                                                      path_boundary.delta_s(),
                                                                      path_boundary.start_s());
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));

      if (FLAGS_use_front_axe_center_in_path_planning) {
        // 如果在路径规划中使用前轴中心，转换路径点参考从前轴到后轴
        auto discretized_path = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                                                path_data));
        path_data.SetDiscretizedPath(discretized_path);      // 设置离散化路径
      }
      path_data.set_path_label(path_boundary.label());       // regular/lane_change
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());

      candidate_path_data->push_back(std::move(path_data));  // 添加候选路径
      
      AINFO << " path_label : " << path_boundary.label() ;
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneChangePath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  std::vector<PathData> valid_path_data;
  for (auto& curr_path_data : *candidate_path_data) {
    AINFO << " AssessPath ";

    // 遍历候选路径数据，检查路径是否有效。其中会判断路径是否为空、路径是否远离参考线、路径是否远离道路、路径是否与静态障碍物碰撞、路径终点是否在逆向的临近车道上。
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      // 如果当前路径是有效的常规路径
      SetPathInfo(&curr_path_data);
      PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data); // 剪除尾部超出车道的点
      if (curr_path_data.Empty()) {
        AINFO << "lane change path is empty after trimed";
        continue;
      }
      valid_path_data.push_back(curr_path_data);
    }
  }
  if (valid_path_data.empty()) {
    AINFO << "All lane change path are not valid";
    return false;
  }

  *final_path = valid_path_data[0];
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

void LaneChangePath::UpdateLaneChangeStatus() {
  std::string change_lane_id;
  auto* prev_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_change_lane();
  double now = Clock::NowInSeconds();

  // Init lane change status
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
    return;
  }

  // 参考线数量为1，之前是变道状态，本周期结束变道状态。结束变道的时候，可能刚过线，l = -1.61左右
  bool has_change_lane = frame_->reference_line_info().size() > 1;
  if (!has_change_lane) {
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                   prev_status->path_id());
    }
    return;
  }

  // 参考线数量大于1  has change lane
  if (reference_line_info_->IsChangeLanePath()) {
    const auto* history_frame = injector_->frame_history()->Latest();
    if (!CheckLastFrameSucceed(history_frame)) { // 上一次规划存在速度回退，即变道失败
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FAILED, change_lane_id);
      is_exist_lane_change_start_position_ = false;
      return;
    }

    // 根据动态障碍物，计算变道是否安全
    is_clear_to_change_lane_ = IsClearToChangeLane(reference_line_info_);
    change_lane_id = reference_line_info_->Lanes().Id();
    AINFO << "change_lane_id " << change_lane_id;
    AINFO << "is_clear_to_change_lane_ " << is_clear_to_change_lane_;

    // 跟新变道状态
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      if (now - prev_status->timestamp() > config_.change_lane_fail_freeze_time()) {  // 大于变道失败冷却时间1s，允许再次进入变道
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        ADEBUG << "change lane again after failed";
      }
      return;
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      if (now - prev_status->timestamp() > config_.change_lane_success_freeze_time()) {// 大于变道成功（首次激活也是）冷却时间3s，允许再次进入变道
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        AINFO << "change lane again after success";
      }
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) { // 变道过程中，如果需要变道的方向改变，结束变道
      if (prev_status->path_id() != change_lane_id) {
        AINFO << "change_lane_id" << change_lane_id << "prev"
              << prev_status->path_id();
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, prev_status->path_id());
      }
    }
  }
}

bool LaneChangePath::IsClearToChangeLane(ReferenceLineInfo* reference_line_info) {
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();
  double ego_v = std::abs(reference_line_info->vehicle_state().linear_velocity());

  for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
    // 不考虑虚拟障碍物和静态障碍物
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    // 遍历所有动态障碍物
    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    // 找到障碍物Boundingbox 4个角的sl
    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      apollo::common::SLPoint sl_point;
      reference_line_info->reference_line().XYToSL(p, &sl_point);

      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }

    // 计算障碍物中心处的道路左右宽度
    if (reference_line_info->IsChangeLanePath()) {
      double left_width(0), right_width(0);
      reference_line_info->mutable_reference_line()->GetLaneWidth((start_s + end_s) * 0.5, 
                                                                  &left_width, &right_width);
      if (end_l < -right_width || start_l > left_width) {
        continue;
      }
    }

    // Raw estimation on whether same direction with ADC or not based on prediction trajectory
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction = obstacle->Trajectory().trajectory_point(0).path_point().theta();
      const auto& vehicle_state = reference_line_info->vehicle_state();
      double vehicle_moving_direction = vehicle_state.heading();

      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction = common::math::NormalizeAngle(vehicle_moving_direction + M_PI);  // 倒车，航向角换180°
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
                                  obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < (M_PI / 2.0); // 自车与障碍物方向角度相差在pi/2之内，两车同向
    }

    // 设置变道过程中，前后的安全距离
    // TODO(All) move to confs
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    if (same_direction) { // 与自车同向
      kForwardSafeDistance = std::fmax(kForwardMinSafeDistanceOnSameDirection,
                                       (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance = std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                                        (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {              // 与自车相向
      kForwardSafeDistance = std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                                       (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    // 判断变道是否安全
    AINFO << "obstacle_distance  " << ego_start_s << " - "<< end_s;
    AINFO << "obstacle_distance  " << start_s << " - "<< ego_end_s;
    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {  // ? 

      reference_line_info->path_decision()->Find(obstacle->Id())->SetLaneChangeBlocking(true); // 无法安全变道
      AINFO << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      reference_line_info->path_decision()->Find(obstacle->Id())->SetLaneChangeBlocking(false); // 可以安全变道
    }
  }
  return true;
}

void LaneChangePath::GetLaneChangeStartPoint(const ReferenceLine& reference_line, 
                                             double adc_frenet_s,
                                             common::math::Vec2d* start_xy) {
  double lane_change_start_s = config_.lane_change_prepare_length() + adc_frenet_s;
  common::SLPoint lane_change_start_sl;
  lane_change_start_sl.set_s(lane_change_start_s);
  lane_change_start_sl.set_l(0.0);
  reference_line.SLToXY(lane_change_start_sl, start_xy);
}

void LaneChangePath::GetBoundaryFromLaneChangeForbiddenZone(PathBoundary* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);

  if (is_clear_to_change_lane_) {
    is_exist_lane_change_start_position_ = false;
    return;
  }
  AINFO << "is_exist_lane_change_start_position_  " << is_exist_lane_change_start_position_;

  // ?  变道起始点计算，变道不安全的时候，没有地方对 lane_change_start_xy_ 赋值
  double lane_change_start_s = 0.0;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  // If there is a pre-determined lane-change starting position, then use it; otherwise, decide one.
  if (is_exist_lane_change_start_position_) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_start_xy_, &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    lane_change_start_s = config_.lane_change_prepare_length() + init_sl_state_.first[0];  // 5 + s0

    // Update the lane_change_start_xy_ decided by lane_change_start_s
    GetLaneChangeStartPoint(reference_line, init_sl_state_.first[0],
                            &lane_change_start_xy_);
  }
  AINFO << "lane_change_start_s  " << lane_change_start_s;

  // Remove the target lane out of the path-boundary, up to the decided S.
  if (lane_change_start_s < init_sl_state_.first[0]) {
    // If already passed the decided S, then return.
    return;
  }

  double adc_half_width = VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    AINFO << "curr_s                 " << curr_s;

    if (curr_s > lane_change_start_s) {
      break;
    }

    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;

      AINFO << "offset_to_lane_center  " << offset_to_lane_center;
      AINFO << "curr_lane_left_width   " << curr_lane_left_width;
      AINFO << "curr_lane_right_width  " << curr_lane_right_width;
    }

    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    (*path_bound)[i].l_lower.l = init_sl_state_.second[0] > curr_lane_left_width
                                 ? curr_lane_left_width + adc_half_width
                                 : (*path_bound)[i].l_lower.l;
    (*path_bound)[i].l_lower.l = std::fmin((*path_bound)[i].l_lower.l, init_sl_state_.second[0] - 0.1);

    (*path_bound)[i].l_upper.l = init_sl_state_.second[0] < -curr_lane_right_width
                                 ? -curr_lane_right_width - adc_half_width
                                 : (*path_bound)[i].l_upper.l;
    (*path_bound)[i].l_upper.l = std::fmax((*path_bound)[i].l_upper.l, init_sl_state_.second[0] + 0.1);

    AINFO << "l_upper                " << (*path_bound)[i].l_upper.l;
    AINFO << "l_lower                " << (*path_bound)[i].l_lower.l;
  }
}

void LaneChangePath::UpdateStatus(double timestamp,
                                  ChangeLaneStatus::Status status_code,
                                  const std::string& path_id) {
  auto* lane_change_status = injector_->planning_context()
                                      ->mutable_planning_status()
                                      ->mutable_change_lane();
  AINFO << "lane change update from" << lane_change_status->DebugString()
        << "to";
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
  AINFO << lane_change_status->DebugString();
}

bool LaneChangePath::HysteresisFilter(const double obstacle_distance,
                                      const double safe_distance,
                                      const double distance_buffer,
                                      const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

void LaneChangePath::SetPathInfo(PathData* const path_data) {
  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(*path_data, 
                                                   PathData::PathPointType::IN_LANE, 
                                                   &path_decision);
  // Go through every path_point, and add in-lane/out-of-lane info.
  const auto& discrete_path = path_data->discretized_path();
  SLBoundary ego_sl_boundary;
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
      ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
      continue;
    }
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s = (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
    if (reference_line_info_->reference_line().GetLaneWidth(middle_s, 
                                                            &lane_left_width, 
                                                            &lane_right_width)) {
      // Rough sl boundary estimate using single point lane width
      double back_to_inlane_extra_buffer = 0.2;
      // For lane-change path, only transitioning part is labeled as
      // out-of-lane.
      if (ego_sl_boundary.start_l() > lane_left_width ||
          ego_sl_boundary.end_l() < -lane_right_width) {
        // This means that ADC hasn't started lane-change yet.
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else if (ego_sl_boundary.start_l() > -lane_right_width + back_to_inlane_extra_buffer &&
                 ego_sl_boundary.end_l() < lane_left_width - back_to_inlane_extra_buffer) {
        // This means that ADC has safely completed lane-change with margin.
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
      } else {
        // ADC is right across two lanes.
        std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_FORWARD_LANE;
      }
    } else {
      AERROR << "reference line not ready when setting path point guide";
      return;
    }
  }
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool LaneChangePath::CheckLastFrameSucceed(const apollo::planning::Frame* const last_frame) {
  if (last_frame) {
    for (const auto& reference_line_info : last_frame->reference_line_info()) {
      if (!reference_line_info.IsChangeLanePath()) {
        continue;
      }
      const auto history_trajectory_type = reference_line_info.trajectory_type();
      if (history_trajectory_type == ADCTrajectory::SPEED_FALLBACK) { // 速度回退
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
