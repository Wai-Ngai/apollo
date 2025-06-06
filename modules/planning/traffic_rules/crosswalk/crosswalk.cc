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

#include "modules/planning/traffic_rules/crosswalk/crosswalk.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/planning/planning_base/proto/planning_status.pb.h"
#include "cyber/time/clock.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;

using CrosswalkToStop = std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
using CrosswalkStopTimer = std::unordered_map<std::string, std::unordered_map<std::string, double>>;


bool Crosswalk::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<CrosswalkConfig>(&config_);
}

Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // 检查是否存在人行横道区域
  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    return Status::OK();
  }

  // 为每个障碍物做标记，障碍物存在时，无人车应该停车还是直接驶过
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto* mutable_crosswalk_status = injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_crosswalk();

  auto* path_decision = reference_line_info->path_decision();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  CrosswalkToStop crosswalks_to_stop;

  // read crosswalk_stop_timer from saved status
  CrosswalkStopTimer crosswalk_stop_timer;
  std::unordered_map<std::string, double> stop_times;
  for (const auto& stop_time : mutable_crosswalk_status->stop_time()) {
    stop_times.emplace(stop_time.obstacle_id(), stop_time.stop_timestamp_sec());
  }
  crosswalk_stop_timer.emplace(mutable_crosswalk_status->crosswalk_id(),
                               stop_times);

  const auto& finished_crosswalks = mutable_crosswalk_status->finished_crosswalk();

  // 1. 检查在每个人行横道区域内，是否存在障碍物需要无人车停车让行
  const auto& reference_line = reference_line_info->reference_line();
  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    std::string crosswalk_id = crosswalk_ptr->id().id();

    // skip crosswalk if master vehicle body already passes the stop line
    if (adc_front_edge_s - crosswalk_overlap->end_s > config_.min_pass_s_distance()) {  // 车头驶过人行横道一定距离，min_pass_s_distance：1.0
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }

    // check if crosswalk already finished
    if (finished_crosswalks.end() != std::find(finished_crosswalks.begin(),
                                               finished_crosswalks.end(),
                                               crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id << "] crosswalk_end_s["
             << crosswalk_overlap->end_s << "] finished already";
      continue;
    }

    std::vector<std::string> pedestrians;
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      const double stop_deceleration = util::GetADCStopDeceleration(injector_->vehicle_state(), 
                                                                    adc_front_edge_s,
                                                                    crosswalk_overlap->start_s);

      bool stop = CheckStopForObstacle(reference_line_info, crosswalk_ptr,
                                       *obstacle, stop_deceleration);

      const std::string& obstacle_id = obstacle->Id();
      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);

      // update stop timestamp on static pedestrian for watch timer
      const bool is_on_lane = reference_line.IsOnLane(obstacle->PerceptionSLBoundary());
      if (stop && !is_on_lane &&
          crosswalk_overlap->start_s - adc_front_edge_s <= config_.start_watch_timer_distance()) {
        // check on stop timer for static pedestrians/bicycles
        // if NOT on_lane ahead of adc
        const double kMaxStopSpeed = 0.3;
        auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                         perception_obstacle.velocity().y());
        if (obstacle_speed <= kMaxStopSpeed) {
          if (crosswalk_stop_timer[crosswalk_id].count(obstacle_id) < 1) {
            // add timestamp
            ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                   << "] timestamp[" << Clock::NowInSeconds() << "]";
            crosswalk_stop_timer[crosswalk_id].insert(
                {obstacle_id, Clock::NowInSeconds()});
          } else {
            double stop_time = Clock::NowInSeconds() -
                               crosswalk_stop_timer[crosswalk_id][obstacle_id];
            ADEBUG << "stop_time: obstacle_id[" << obstacle_id << "] stop_time["
                   << stop_time << "]";
            if (stop_time >= config_.stop_timeout()) {
              stop = false;
            }
          }
        }
      }

      if (stop) {
        pedestrians.push_back(obstacle_id);
        ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      } else {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      }
    }

    if (!pedestrians.empty()) {
      crosswalks_to_stop.emplace_back(crosswalk_overlap, pedestrians);
      ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
    }
  }

  // 2. 对那些影响无人车行驶的障碍物构建虚拟墙障碍物类以及设置停车标签
  double min_s = std::numeric_limits<double>::max();
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    // build stop decision
    const auto* crosswalk_overlap = crosswalk_to_stop.first;

    ADEBUG << "BuildStopDecision: crosswalk[" << crosswalk_overlap->object_id
           << "] start_s[" << crosswalk_overlap->start_s << "]";

    std::string virtual_obstacle_id = CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;

    util::BuildStopDecision(virtual_obstacle_id, crosswalk_overlap->start_s,
                            config_.stop_distance(), StopReasonCode::STOP_REASON_CROSSWALK,
                            crosswalk_to_stop.second, Getname(), frame, reference_line_info);
                            
    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      min_s = crosswalk_to_stop.first->start_s;
    }
  }

  if (firsts_crosswalk_to_stop) {
    // update CrosswalkStatus
    std::string crosswalk = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk);
    mutable_crosswalk_status->clear_stop_time();
    for (const auto& timer : crosswalk_stop_timer[crosswalk]) {
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      stop_time->set_obstacle_id(timer.first);
      stop_time->set_stop_timestamp_sec(timer.second);
      ADEBUG << "UPDATE stop_time: id[" << crosswalk << "] obstacle_id["
             << timer.first << "] stop_timestamp[" << timer.second << "]";
    }

    // update CrosswalkStatus.finished_crosswalk
    mutable_crosswalk_status->clear_finished_crosswalk();
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: " << crosswalk_overlap->object_id;
      }
    }
  }

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
}

bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps = reference_line_info->reference_line().map_path().crosswalk_overlaps();
  
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::CheckStopForObstacle(ReferenceLineInfo* const reference_line_info,
                                     const CrosswalkInfoConstPtr crosswalk_ptr, 
                                     const Obstacle& obstacle,
                                     const double stop_deceleration) {
  CHECK_NOTNULL(reference_line_info);

  std::string crosswalk_id = crosswalk_ptr->id().id();

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& obstacle_id = obstacle.Id();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // check type
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return false;
  }

  // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(config_.expand_s_distance());
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);

  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "]: not in crosswalk expanded area";
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();

  common::SLPoint obstacle_sl_point;
  reference_line.XYToSL(perception_obstacle.position(), &obstacle_sl_point);
  auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle_sl_boundary.start_l()),
               std::fabs(obstacle_sl_boundary.end_l()));

  const bool is_on_lane =
      reference_line.IsOnLane(obstacle.PerceptionSLBoundary());
  const bool is_on_road =
      reference_line.IsOnRoad(obstacle.PerceptionSLBoundary());
  const bool is_path_cross = !obstacle.reference_line_st_boundary().IsEmpty();

  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] crosswalk_id[" << crosswalk_id << "] obstacle_l["
         << obstacle_sl_point.l() << "] within_expanded_crosswalk_area["
         << in_expanded_crosswalk << "] obstacle_l_distance["
         << obstacle_l_distance << "] on_lane[" << is_on_lane << "] is_on_road["
         << is_on_road << "] is_path_cross[" << is_path_cross << "]";

  bool stop = false;
  if (obstacle_l_distance >= config_.stop_loose_l_distance()) {  // stop_loose_l_distance: 5.0m
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
    //     STOP only if paths crosses
    // 如果横向距离大于疏松距离。如果轨迹相交，那么需要停车；反之直接驶过(此时轨迹相交无所谓，因为横向距离比较远)
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
  } else if (obstacle_l_distance <= config_.stop_strict_l_distance()) {// stop_strick_l_distance: 4.0m
    if (is_on_road) {
      // (2) when l_distance <= strict_l_distance + on_road
      //     always STOP
      // 如果横向距离小于紧凑距离。如果障碍物在参考线或者轨迹相交，那么需要停车；反之直接驶过
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_end_edge_s[ " << adc_end_edge_s << "] crosswalk_id["
               << crosswalk_id << "] ON_ROAD";
      }
    } else {
      // (3) when l_distance <= strict_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "] PATH_CRSOSS";
      } else {
        // (4) when l_distance <= strict_l_distance
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        const auto adc_path_point =
            Vec2d(injector_->ego_info()->start_point().path_point().x(),
                  injector_->ego_info()->start_point().path_point().y());
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        auto obs_to_adc = adc_path_point - ovstacle_position;
        const double kEpsilon = 1e-6;
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "] MOVING_TOWARD_ADC";
        }
      }
    }
  } else { // - 如果横向距离在紧凑距离和疏松距离之间，直接停车
    // (4) when l_distance is between loose_l and strict_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
           << "] USE_PREVIOUS_DECISION";
  }

  // check stop_deceleration
  if (stop) {
    if (stop_deceleration >= config_.max_stop_deceleration()) { // 最后可以计算无人车的加速度(是否来的及减速，若无人车速度很快减速不了，那么干脆直接驶过)
      if (obstacle_l_distance > config_.stop_strict_l_distance()) {
        // SKIP when stop_deceleration is too big but safe to ignore
        stop = false;
      }
      AWARN << "crosswalk_id[" << crosswalk_id << "] stop_deceleration["
            << stop_deceleration << "]";
    }
  }

  return stop;
}

}  // namespace planning
}  // namespace apollo
