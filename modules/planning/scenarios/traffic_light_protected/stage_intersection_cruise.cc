/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/traffic_light_protected/stage_intersection_cruise.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

StageResult TrafficLightProtectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightProtectedStageIntersectionCruise plan error";
  }

  bool stage_done = CheckDone(*frame, injector_->planning_context(), true);

  if (stage_done) { // 如果通过交叉路口，本Stage完成。恢复到默认场景LANE_FOLLOW
    return FinishStage();
  }
  // 否则，进入本Stage的Running
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult TrafficLightProtectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
