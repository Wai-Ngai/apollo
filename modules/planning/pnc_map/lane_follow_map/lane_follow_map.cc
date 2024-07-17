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
 * @file lane_follow_map.cc
 **/

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/common_msgs/map_msgs/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

LaneFollowMap::LaneFollowMap() : hdmap_(hdmap::HDMapUtil::BaseMapPtr()) {}

bool LaneFollowMap::CanProcess(const planning::PlanningCommand &command) const {
  return command.has_lane_follow_command();
}

//routing::LaneWaypoint转换为hdmap::lanewaypoint结构体
hdmap::LaneWaypoint LaneFollowMap::ToLaneWaypoint(const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return hdmap::LaneWaypoint(lane, waypoint.s());
}

// routing::LaneSegment转换成hdmap::LaneSegment结构体
hdmap::LaneSegment LaneFollowMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return hdmap::LaneSegment(lane, segment.start_s(), segment.end_s());
}

// 更新下个Routing Waypoint序号
void LaneFollowMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  // 如果是起始点-1，则下个点next_routing_waypoint_index_=0
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  // 如果大于route_indices_的数量，则返回最后一个查询点
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }

  // 情况1. 车道倒车，后向查找，下一个查询点waypoint对应的索引查找
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index > cur_index) {// 不一定是倒车，也可能是按原路返回
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index == cur_index &&
         adc_waypoint_.s < routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  
  // 情况2. 车道前进，前向查找，下一个查询点waypoint对应的索引查找
  // search forwards
  // 第1步，查找最近包含有waypoint的LaneSegment 
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index < cur_index) {
    ++next_routing_waypoint_index_;
  }
  // 第2步，查找下一个最近的waypoint对应的索引
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index == routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >= routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  // 如果超过范围，则返回最大值  
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> LaneFollowMap::FutureRouteWaypoints() const {
  const auto &waypoints =
      last_command_.lane_follow_command().routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void LaneFollowMap::GetEndLaneWayPoint(std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (!last_command_.has_lane_follow_command() ||
      !last_command_.lane_follow_command().has_routing_request()) {
    end_point = nullptr;
    return;
  }
  const auto &routing_request =
      last_command_.lane_follow_command().routing_request();
  if (routing_request.waypoint().size() < 1) {
    end_point = nullptr;
    return;
  }
  end_point = std::make_shared<routing::LaneWaypoint>();
  end_point->CopyFrom(*(routing_request.waypoint().rbegin()));
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetLaneById(const hdmap::Id &id) const {
  if (nullptr == hdmap_) {
    return nullptr;
  }
  return hdmap_->GetLaneById(id);
}

bool LaneFollowMap::IsValid(const planning::PlanningCommand &command) const {
  if (!CanProcess(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  const int num_road = routing.road_size();
  if (num_road == 0) {
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

// 完成对rang_lane_ids_更新，以及起始range_start_，结束点range_end_进行更新
void LaneFollowMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;

  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool LaneFollowMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  if (!IsValid(last_command_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }

  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) > FLAGS_replan_lateral_distance_threshold +         // 0.5
                                                             FLAGS_replan_longitudinal_distance_threshold)) {  // 2.5
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  // Step 1. 计算当前车辆在对应车道上的投影adc_waypoint_
  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }

  // Step 2. 计算车辆投影点所在LaneSegment在route_indices_`的索引`route_index_`
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }
  ADEBUG << "adc_waypoint_" << adc_waypoint_.DebugString() << "route_index"
         << route_index;

  // Track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index);  // 从当前车辆的索引开始，向后查找最近的查询点waypoint

  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);  //获取剩余的routing路线

  if (routing_waypoint_index_.empty()) { //没有routing信息
    AERROR << "No routing waypoint index.";
    return false;
  }

  // Step 3.到达目的地标志
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) { //如果到了最后一个点，则停车标志位置true
    stop_for_destination_ = true;
  }
  return true;
}

bool LaneFollowMap::UpdatePlanningCommand(const planning::PlanningCommand &command) {
  
  // 检查cammand中是否有RoutingResponse
  if (!CanProcess(command)) {
    AERROR << "Command cannot be processed by LaneFollowMap!";
    return false;
  }
  if (!PncMapBase::UpdatePlanningCommand(command)) {
    return false;
  }

  // 1.更新路由信息
  const auto &routing = command.lane_follow_command();

  //清除range_lane_ids_，route_indices_，all_lane_ids_三个成员变量的内容
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();

  // step 1.响应结果剥离：遍历所有的 RoadSegment -> Passage -> LaneSegment
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);

    for (int passage_index = 0; passage_index < road_segment.passage_size(); ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);

      for (int lane_index = 0; lane_index < passage.segment_size(); ++lane_index) {

        // 将每条lane的id存进all_lane_ids这个数组中
        all_lane_ids_.insert(passage.segment(lane_index).id());

        // 将每个hdmap::lane_segment及{road_index, passage_index, lane_index} 存进route_indices_中
        route_indices_.emplace_back(); // 向量末尾添加一个默认构造的 RouteIndex 对象。调用 RouteIndex 结构体的默认构造函数
        route_indices_.back().segment = ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;

  UpdateRoutingRange(adc_route_index_);

  // step 2.查询点处理 : 寻找在waypoints和route_indices_，将路径请求信息装进routing_waypoint_index_
  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();  // 查询点：路径起点和终点，途经点
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }

  // 计算这两个waypoint分别在上述的哪些LaneSegment中
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           hdmap::RouteSegments::WithinLaneSegment(route_indices_[j].segment,  // 判断这个waypoint是否在当前LaneSegment中
                                                   request_waypoints.Get(i))) {
      // 保存查询点的路由结果中的id,s,index
      routing_waypoint_index_.emplace_back(hdmap::LaneWaypoint(route_indices_[j].segment.lane, request_waypoints.Get(i).s()),
                                           j);
      ++i;
    }
  }
  adc_waypoint_ = hdmap::LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

int LaneFollowMap::SearchForwardWaypointIndex(int start, 
                                              const hdmap::LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (i < static_cast<int>(route_indices_.size()) &&
         !hdmap::RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                  waypoint)) {
    ++i;
  }
  return i;
}

int LaneFollowMap::SearchBackwardWaypointIndex(int start, 
                                               const hdmap::LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !hdmap::RouteSegments::WithinLaneSegment(
                       route_indices_[i].segment, waypoint)) {
    --i;
  }
  return i;
}

int LaneFollowMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int LaneFollowMap::GetWaypointIndex(const hdmap::LaneWaypoint &waypoint) const {
  // 根据车辆所在位置，确认其在rout point的序号
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);  // 先向前搜索
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);     // 前向遍历完整个容器，没搜到，向后搜索
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

// passage表达的数据内容和RouteSegments其实是类似的，RouteSegments就是将Passage中的一条条lane，存进其数组里，并记录起始终止的s值。
bool LaneFollowMap::PassageToSegments(routing::Passage passage,
                                      hdmap::RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  // 遍历passage中所有的segment
  for (const auto &lane : passage.segment()) {
    //根据id获取对应的指针
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    // 调用构造函数，创建对象，传入lane,start_s,end_s
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

std::vector<int> LaneFollowMap::GetNeighborPassages(const routing::RoadSegment &road, 
                                                    int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());

  std::vector<int> result;

  // 根据车辆位置序号，获取passage信息
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);

  // 情况1：当前通道(Passage)是直行道，无法变道，那么直接返回车辆所在的车道
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }

  // 情况2：当前通道已经准备退出，则表示即将驶入下一个passage，不需要变道，直接返回车辆所在的车道
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }

  hdmap::RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {// 获取当前车道RouteSegments
    AERROR << "Failed to convert passage to segments";
    return result;
  }

  // 情况3：如果下一个必经查询点在当前passage上，不需要变道，直接返回车辆所在的车道
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }

  // 情况4：车辆在左转车道或者右转车道，从高精地图hd map中查询当前车道对应左侧或者右侧的所有车道线，
  // 然后去和当前RoadSegment.passage()去做对比，找到两者共同包含的车道，就是最终的邻接车道。
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) { // 当前passage是左转通道
    for (const auto &segment : source_segments) {           // 查询当前Passage中每个LaneSegment所在车道的邻接 左车道
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) { // 当前passage是右转通道
    for (const auto &segment : source_segments) {                   // 查询当前Passage中每个LaneSegment所在车道的邻接 右车道
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  //neighbor_lanes中如果有target_passage，则将序号放进result中
  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {  // 查询当前RoadSegment中所有Passage::LaneSegment的所属车道，有交集就添加到结果中
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}

bool LaneFollowMap::GetRouteSegments(const VehicleState &vehicle_state,
                                     std::list<hdmap::RouteSegments> *const route_segments) {
  double look_forward_distance = LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;  

  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

bool LaneFollowMap::GetRouteSegments(const VehicleState &vehicle_state, 
                                     const double backward_length,  // 50m
                                     const double forward_length,   // 180 or 250m
                                     std::list<hdmap::RouteSegments> *const route_segments) {
  // Step 1.更新pnc map中无人车状态： 无人车在哪个LaneSegment中，距离无人车最近的下一个查询点waypoint的信息
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change lane
  // 如果adc_waypoint没有道路信息，或者_adc_route_index_无效，返回错误信息
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }

  // Step 2.计算临近通道
  const auto &route_index = route_indices_[adc_route_index_].index; // `{road_index, passage_index, lane_index}`
  const int road_index = route_index[0];           // 道路序号
  const int passage_index = route_index[1];        // passage序号
  const auto &road = last_command_.lane_follow_command().road(road_index);    // 根据道路序号获取道路信息
  
  // Raw filter to find all neighboring passages
  // 获取当前位置所有相邻车道的index，后面会对所有drive_passages进行帅选，不符合条件的就会剔除
  auto drive_passages = GetNeighborPassages(road, passage_index);
  
  // Step 3.创建车辆当前可行驶区域 route_segments
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index); // 根据车道index获取相邻车道信息

    hdmap::RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) { // 将passage转换成RouteSegments
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
      // Step 3.1 将当前车辆的坐标投影到Passage
    const PointENU nearest_point = index == passage_index
                                  ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
                                  : PointFactory::ToPointENU(adc_state_);

    common::SLPoint sl;
    hdmap::LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
  
      // Step 3.2 检查Passage是否可驶入
    if (index != passage_index) {     // 非当前车道
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
  
      // Step 3.3 生成RouteSegmens
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();

      //对segment进行拓展，前向增加forward_length，后向增加backward_length
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
  
      // Step 3.4 设置RouteSegments属性
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());            // 是否可以退出通道(最后一段Segment决定)
    route_segments->back().SetNextAction(passage.change_lane_type()); // 换道方式（左换道，右换道）(最后一段Segment决定)
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);//设置是否停车的标志位
   
    // 设置上时刻的状态
    if (index == passage_index) { // 当前车道
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD); //设置前置动作为forward，直行
    } else if (sl.l() > 0) {     // 如果当前车辆在passage左侧，那么车辆肯定需要向右变道到passage
      route_segments->back().SetPreviousAction(routing::RIGHT);//设置前置动作为right，右转
    } else {                     // 如果当前车辆在passage右侧，那么车辆肯定需要向左变道到passage
      route_segments->back().SetPreviousAction(routing::LEFT);//设置前置动作为left，左转
    }
  }
  return !route_segments->empty();
}

bool LaneFollowMap::GetNearestPointFromRouting(const common::VehicleState &state,
                                               hdmap::LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;

  for (auto lane_id : all_lane_ids_) {
    hdmap::Id id = hdmap::MakeMapId(lane_id);
    auto lane = hdmap_->GetLaneById(id);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  // 在有效lanes中查找与车辆位置最近的点，并将距离最小的lane作为车辆所在的lane
  // Get nearest_waypoints for current position
  std::vector<hdmap::LaneWaypoint> valid_way_points;

  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in range" << lane->id().id();
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) { // 车辆定位xy投影点sl
        continue;
      }
      ADEBUG << lane->id().id() << "," << s << "," << l;
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
      double lane_heading = lane->Heading(s);
      if (std::fabs(common::math::AngleDiff(lane_heading, state.heading())) > M_PI_2 * 1.5) {
        continue;
      }
    }

    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  if (valid_way_points.empty()) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
    return false;
  }

  // 根据当前车辆坐标(x,y)以及速度方向heading，去高精地图hd map查询车辆附近的同向车道
  // find closest lane that satisfy vehicle heading
  int closest_index = -1;
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();

  for (size_t i = 0; i < valid_way_points.size(); i++) {
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);

    if (std::abs(common::math::AngleDiff(lane_heading, vehicle_heading)) > M_PI_2 * 1.5) { // 判断车道与车辆是否同向
      continue;
    }
    if (std::fabs(valid_way_points[i].l) < distance) {  // 找到距离最近的车道
      distance = std::fabs(valid_way_points[i].l);
      closest_index = i;
    }
  }
  if (closest_index == -1) {
    AERROR << "Can not find nearest waypoint. vehicle heading:"
           << vehicle_heading << "lane heading:" << lane_heading;
    return false;
  }

  waypoint->lane = valid_way_points[closest_index].lane;
  waypoint->s = valid_way_points[closest_index].s;
  waypoint->l = valid_way_points[closest_index].l;
  return true;
}

//求后一个连接lane id
hdmap::LaneInfoConstPtr LaneFollowMap::GetRouteSuccessor(hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

//求前连接的lane id
hdmap::LaneInfoConstPtr LaneFollowMap::GetRoutePredecessor(hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (const auto &route_index : route_indices_) {
    auto &lane = route_index.segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool LaneFollowMap::ExtendSegments(const hdmap::RouteSegments &segments,
                                   const common::PointENU &point,
                                   double look_backward, double look_forward,
                                   hdmap::RouteSegments *extended_segments) {
  common::SLPoint sl;
  hdmap::LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool LaneFollowMap::ExtendSegments(const hdmap::RouteSegments &segments, 
                                   double start_s, double end_s,
                                   hdmap::RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }

  // 1. 前置车道处理
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {                               // 当后向查询起始点小于0，说明需要用到这条lane的前置lane
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;       // 获得passage的第一个LaneSegment的所属车道
    double s = first_segment.start_s;
    double extend_s = -start_s;           // extend_s为需要从前置车道中截取的道路段长度，初始化为-start_s，

    std::vector<hdmap::LaneSegment> extended_lane_segments;

    while (extend_s > kRouteEpsilon) {    // 每次循环(截取)以后extend_s都会减小，直至到0
      if (s <= kRouteEpsilon) {           // s < 0，则需要在查询这条lane对应的前置车道，进行截取
        lane = GetRoutePredecessor(lane); // 获取当前lane的前置车道
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {                           // 如果s > 0，此时就可以从这条前置lane中截取道路段
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s); // 截取道路段
        extend_s -= length;              // 更新extend_s，如果extend_s>0，说明还需要继续寻找前置道路段截取
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }

  // 正常的passage中LaneSegment截取，根据start_s和end_s
  bool found_loop = false;

  // router_s代表已经累计截取到了的LaneSegment长度，如果当前正在截取第3个LaneSegment，那么router_s就是前两个LaneSegment的长度和
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    // 计算当前LaneSegment需要截取的start_s和end_s，非最后一段，start_s和end_s就是这个LaneSegment的start_s和end_s，意味着整段截取
    const double adjusted_start_s = std::max( start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s = std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    
    if (adjusted_start_s < adjusted_end_s) {
      // 有前置车道的，如果前置最后一段的前置车道和当前LaneSegment的车道相同，那么需要合并(修改end_s即可)；否则新建一段加入list
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() == lane_segment.lane->id().id()) {

        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) == unique_lanes.end()) {

        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s, adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    // 判断是否截取结束，如果结束了那么可以退出，否则就需要继续截取，当最后循环最后一次最后一个LaneSegment还是没有结束，那么就需要
    // 新增加后置车道继续处理
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }

  // 2. 后接车道处理
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {       // 仍然有未被截取的道路段(长度还没满足)
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {               // 查找最后一个LaneSegment对应的车道，继续从该车道截取
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {              // 如果最后一个LaneSegment对应的车道截取完了，还没达到长度要求，虚招这个车道的后接车道，继续截取
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }
  return true;
}

// RouteSegments离散化MapPathPoint
void LaneFollowMap::AppendLaneToPoints(hdmap::LaneInfoConstPtr lane, 
                                       const double start_s, const double end_s,
                                       std::vector<hdmap::MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {         // 封装中间点point
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           hdmap::LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();

      if (start_s > accumulate_s && start_s < next_accumulate_s) { // 封装段起点waypoint
        points->emplace_back(segment.start() + segment.unit_direction() * (start_s - accumulate_s),
                             lane->headings()[i],
                             hdmap::LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {     // 封装段终点waypoint
        points->emplace_back(segment.start() + segment.unit_direction() * (end_s - accumulate_s),
                             lane->headings()[i], 
                             hdmap::LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

}  // namespace planning
}  // namespace apollo
