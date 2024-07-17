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

#include "modules/planning/traffic_rules/backside_vehicle/backside_vehicle.h"

#include <memory>
#include <string>

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;


bool BacksideVehicle::Init(
    const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<BacksideVehicleConfig>(&config_);
}

void BacksideVehicle::MakeLaneKeepingObstacleDecision(const SLBoundary& adc_sl_boundary, 
                                                      PathDecision* path_decision,
                                                      const VehicleState& vehicle_state) {
  // 从这里可以看到，对于后车的处理主要是忽略
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  const double adc_length_s = adc_sl_boundary.end_s() - adc_sl_boundary.start_s(); // 计算"车长"
  for (const auto* obstacle : path_decision->obstacles().Items()) {  // 对于每个与参考线有重叠的障碍物进行规则设置

    if (obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.start_s()) { // 前车在这里不做考虑，由前车情况处理FRONT_VEHICLE来完成
      // don't ignore such vehicles.
      continue;
    }

    if (obstacle->IsCautionLevelObstacle() &&
        obstacle->PerceptionSLBoundary().start_s() >= adc_sl_boundary.start_s()) {
      // don't ignore such vehicles.
      continue;
    }

    if (obstacle->reference_line_st_boundary().IsEmpty()) { // 参考线上没有障碍物运动轨迹，直接忽略
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region", obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region", obstacle->Id(), ignore);
      continue;
    }
    // Ignore the car comes from back of ADC
    if (obstacle->reference_line_st_boundary().min_s() < -adc_length_s) { // 忽略从无人车后面来的车辆
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc", obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc", obstacle->Id(), ignore);
      continue;
    }

    const double lane_boundary = config_.backside_lane_width();
    if (obstacle->PerceptionSLBoundary().start_s() < adc_sl_boundary.end_s()) { // 忽略后面不会超车的车辆
      if (obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
          obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
        continue;
      }
      path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s", obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s", obstacle->Id(), ignore);
      continue;
    }

    if (PredictionLineOverlapEgo(*obstacle, vehicle_state)) {
      ADEBUG << "Prediction Line Overlap Ego Obstacle " << obstacle->Id();
      path_decision->AddLongitudinalDecision("backside_vehicle prediction line overlap ego", obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle prediction line overlap ego", obstacle->Id(), ignore);
      continue;
    }
  }
}

Status BacksideVehicle::ApplyRule(Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const VehicleState& vehicle_state = reference_line_info->vehicle_state();
  // The lane keeping reference line.
  if (reference_line_info->Lanes().IsOnSegment()) {
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision,
                                    vehicle_state);
  }
  return Status::OK();
}

bool BacksideVehicle::PredictionLineOverlapEgo(
    const Obstacle& obstacle, const VehicleState& vehicle_state) {
  const auto& trajectory = obstacle.Trajectory().trajectory_point();

  if (trajectory.empty()) return false;

  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  Box2d adc_box({vehicle_state.x(), vehicle_state.y()}, vehicle_state.heading(),
                vehicle_param.length(), vehicle_param.width());
  Polygon2d adc_polygon(adc_box);
  for (auto& point : trajectory) {
    Polygon2d obs_polygon = obstacle.GetObstacleTrajectoryPolygon(point);
    if (adc_polygon.HasOverlap(obs_polygon)) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
