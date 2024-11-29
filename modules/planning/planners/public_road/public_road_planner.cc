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

#include "modules/planning/planners/public_road/public_road_planner.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Status PublicRoadPlanner::Init(const std::shared_ptr<DependencyInjector>& injector,
                               const std::string& config_path) {  // "modules/planning/planning_component/conf/public_road_planner_config.pb.txt"
  Planner::Init(injector, config_path);

  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);

  // 根据配置文件初始化PublicRoadPlanner的场景列表scenario_list
  scenario_manager_.Init(injector, config_);
  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  
  // 计算当前场景
  scenario_manager_.Update(planning_start_point, frame);
  scenario_ = scenario_manager_.mutable_scenario();
  if (!scenario_) {
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "Unknown Scenario");
  }
  AINFO << " Current Scenario : " << scenario_->Name();

  // 每个场景都调用父类的Scenario::Process()，然后根据场景所处的stage，调用该stage的Process()函数
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                                                 ->mutable_planning_data()
                                                 ->mutable_scenario();
    scenario_debug->set_scenario_plugin_type(scenario_->Name());
    scenario_debug->set_stage_plugin_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  // only updates scenario manager when previous scenario's status is STATUS_DONE
  if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_DONE) {
    scenario_manager_.Update(planning_start_point, frame);
  } else if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR, result.GetTaskStatus().error_message());
  }
  return Status(common::OK, result.GetTaskStatus().error_message());
}

}  // namespace planning
}  // namespace apollo
