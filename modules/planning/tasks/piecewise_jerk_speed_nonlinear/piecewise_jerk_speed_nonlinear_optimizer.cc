/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "modules/planning/tasks/piecewise_jerk_speed_nonlinear/piecewise_jerk_speed_nonlinear_optimizer.h"

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/proto/ipopt_return_status.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/piecewise_jerk_speed_nonlinear/piecewise_jerk_speed_nonlinear_ipopt_interface.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

PiecewiseJerkSpeedNonlinearOptimizer::PiecewiseJerkSpeedNonlinearOptimizer()
    : SpeedOptimizer(),
      smoothed_speed_limit_(0.0, 0.0, 0.0),
      smoothed_path_curvature_(0.0, 0.0, 0.0) {}

bool PiecewiseJerkSpeedNonlinearOptimizer::Init(const std::string& config_dir, 
                                                const std::string& name,
                                                const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config_ this task.
  if (SpeedOptimizer::LoadConfig<PiecewiseJerkNonlinearSpeedOptimizerConfig>(&config_)) {
    AERROR << "Failed to load config of PiecewiseJerkSpeedNonlinearOptimizer "
           << Name();
  }
  return true;
}

Status PiecewiseJerkSpeedNonlinearOptimizer::Process(const PathData& path_data, 
                                                     const TrajectoryPoint& init_point,
                                                     SpeedData* const speed_data) {
  // Sanity checks.
  if (speed_data == nullptr) {
    const std::string msg = "Null speed_data pointer";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Speed Optimizer receives empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Step 1 : 检查是否到达终点，如果到达终点不再进行速度规划
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  // Step 2 : 构建速度QP和NLP优化参数、边界
  const auto problem_setups_status = SetUpStatesAndBounds(path_data, *speed_data);
  if (!problem_setups_status.ok()) {
    speed_data->clear();
    return problem_setups_status;
  }

  // Step 3 : 基于QP算法对动态规划的粗ST曲线进行平滑, 用于计算非线性问题的初始解
  std::vector<double> distance;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  const auto qp_start = std::chrono::system_clock::now();

  const auto qp_smooth_status = OptimizeByQP(speed_data, &distance, &velocity, &acceleration);

  const auto qp_end = std::chrono::system_clock::now();
  std::chrono::duration<double> qp_diff = qp_end - qp_start;
  ADEBUG << "speed qp optimization takes " << qp_diff.count() * 1000.0 << " ms";
  if (!qp_smooth_status.ok()) {
    speed_data->clear();
    return qp_smooth_status;
  }

  const bool speed_limit_check_status = CheckSpeedLimitFeasibility();

  if (speed_limit_check_status) {
    // Step 4 : 基于QP算法平滑 路径曲率曲线
    const auto curvature_smooth_start = std::chrono::system_clock::now();
    const auto path_curvature_smooth_status = SmoothPathCurvature(path_data);

    const auto curvature_smooth_end = std::chrono::system_clock::now();
    std::chrono::duration<double> curvature_smooth_diff = curvature_smooth_end - curvature_smooth_start;
    ADEBUG << "path curvature smoothing for nlp optimization takes "
           << curvature_smooth_diff.count() * 1000.0 << " ms";
    if (!path_curvature_smooth_status.ok()) {
      speed_data->clear();
      return path_curvature_smooth_status;
    }

    // Step 5 : 基于QP算法平滑 限速曲线
    const auto speed_limit_smooth_start = std::chrono::system_clock::now();
    const auto speed_limit_smooth_status = SmoothSpeedLimit();

    const auto speed_limit_smooth_end = std::chrono::system_clock::now();
    std::chrono::duration<double> speed_limit_smooth_diff = speed_limit_smooth_end - speed_limit_smooth_start;
    ADEBUG << "speed limit smoothing for nlp optimization takes "
           << speed_limit_smooth_diff.count() * 1000.0 << " ms";
    if (!speed_limit_smooth_status.ok()) {
      speed_data->clear();
      return speed_limit_smooth_status;
    }

    // Step 6 : 基于非线性规划优化ST曲线
    const auto nlp_start = std::chrono::system_clock::now();
    const auto nlp_smooth_status = OptimizeByNLP(&distance, &velocity, &acceleration);

    const auto nlp_end = std::chrono::system_clock::now();
    std::chrono::duration<double> nlp_diff = nlp_end - nlp_start;
    ADEBUG << "speed nlp optimization takes " << nlp_diff.count() * 1000.0 << " ms";
    if (!nlp_smooth_status.ok()) {
      speed_data->clear();
      return nlp_smooth_status;
    }

    // Record speed_constraint
    StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
    auto* speed_constraint = st_graph_data->mutable_st_graph_debug()->mutable_speed_constraint();
    for (int i = 0; i < num_of_knots_; ++i) {
      double t = i * delta_t_;
      speed_constraint->add_t(t);
      speed_constraint->add_upper_bound(smoothed_speed_limit_.Evaluate(0, distance[i]));
    }
  }

  // Step 7 : 提取数据
  speed_data->clear();
  speed_data->AppendSpeedPoint(distance[0], 0.0, velocity[0], acceleration[0],
                               0.0);
  for (int i = 1; i < num_of_knots_; ++i) {
    // Avoid the very last points when already stopped
    if (velocity[i] < 0.0) {
      break;
    }
    speed_data->AppendSpeedPoint(distance[i], delta_t_ * i, 
                                 velocity[i], acceleration[i],
                                 (acceleration[i] - acceleration[i - 1]) / delta_t_);
  }

  // Step 8 : 填充速度点满足轨迹时长要求
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);

  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  RecordDebugInfo(*speed_data, st_graph_data->mutable_st_graph_debug());
  return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SetUpStatesAndBounds(const PathData& path_data, 
                                                                  const SpeedData& speed_data) {
  // Set st problem dimensions
  const StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  // TODO(Jinyun): move to confs
  delta_t_ = 0.1;
  total_length_ = st_graph_data.path_length();
  total_time_ = st_graph_data.total_time_by_conf();
  num_of_knots_ = static_cast<int>(total_time_ / delta_t_) + 1; // 71
  AINFO << " Nonlinear num_of_knots_ :  " << num_of_knots_;

  // Set initial values
  s_init_ = 0.0;
  s_dot_init_ = st_graph_data.init_point().v();
  s_ddot_init_ = st_graph_data.init_point().a();

  // Set s_dot bounary
  s_dot_max_ = std::fmax(FLAGS_planning_upper_speed_limit,
                         st_graph_data.init_point().v());

  // Set s_ddot boundary
  const auto& veh_param = common::VehicleConfigHelper::GetConfig().vehicle_param();
  s_ddot_max_ = veh_param.max_acceleration();                     //  2
  s_ddot_min_ = -1.0 * std::abs(veh_param.max_deceleration());    // -6

  // Set s_dddot boundary
  // TODO(Jinyun): allow the setting of jerk_lower_bound and move jerk config_ to a better place
  s_dddot_min_ = -std::abs(FLAGS_longitudinal_jerk_lower_bound);  // -4
  s_dddot_max_ = FLAGS_longitudinal_jerk_upper_bound;             //  2

  // Set s boundary
  if (config_.use_soft_bound_in_nonlinear_speed_opt()) {   // 若启用软约束
    s_bounds_.clear();
    s_soft_bounds_.clear();

    // TODO(Jinyun): move to confs
    for (int i = 0; i < num_of_knots_; ++i) {
      double curr_t = i * delta_t_;
      double s_lower_bound = 0.0;
      double s_upper_bound = total_length_;
      double s_soft_lower_bound = 0.0;
      double s_soft_upper_bound = total_length_;

      for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
        double s_lower = 0.0;
        double s_upper = 0.0;
        // 获取未被阻塞的s的范围，即s_lower和s_upper
        if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
          continue;
        }

        SpeedPoint sp;
        switch (boundary->boundary_type()) {
          case STBoundary::BoundaryType::STOP:
          case STBoundary::BoundaryType::YIELD:
            s_upper_bound = std::fmin(s_upper_bound, s_upper);
            s_soft_upper_bound = std::fmin(s_soft_upper_bound, s_upper);
            break;

          case STBoundary::BoundaryType::FOLLOW:
            s_upper_bound = std::fmin(s_upper_bound, s_upper);

            if (!speed_data.EvaluateByTime(curr_t, &sp)) {  // 根据DP结果，计算路径点
              const std::string msg = "rough speed profile estimation for soft follow fence failed";
              AERROR << msg;
              return Status(ErrorCode::PLANNING_ERROR, msg);
            }
            s_soft_upper_bound = std::fmin(s_soft_upper_bound,
                                           s_upper - std::min(7.0, 2.5 * sp.v()));
            break;

          case STBoundary::BoundaryType::OVERTAKE:
            s_lower_bound = std::fmax(s_lower_bound, s_lower);
            s_soft_lower_bound = std::fmax(s_soft_lower_bound, s_lower + 10.0);
            break;
          default:
            break;
        }
      }
      if (s_lower_bound > s_upper_bound) {
        const std::string msg = "s_lower_bound larger than s_upper_bound on STGraph";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      s_soft_bounds_.emplace_back(s_soft_lower_bound, s_soft_upper_bound);
      s_bounds_.emplace_back(s_lower_bound, s_upper_bound);
    }
  } else {
    s_bounds_.clear();
    // TODO(Jinyun): move to confs
    for (int i = 0; i < num_of_knots_; ++i) {
      double curr_t = i * delta_t_;
      double s_lower_bound = 0.0;
      double s_upper_bound = total_length_;

      for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
        double s_lower = 0.0;
        double s_upper = 0.0;

        if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
          continue;
        }
        SpeedPoint sp;
        switch (boundary->boundary_type()) {
          case STBoundary::BoundaryType::STOP:
          case STBoundary::BoundaryType::YIELD:
            s_upper_bound = std::fmin(s_upper_bound, s_upper);
            break;

          case STBoundary::BoundaryType::FOLLOW:
            s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0);
            break;

          case STBoundary::BoundaryType::OVERTAKE:
            s_lower_bound = std::fmax(s_lower_bound, s_lower);
            break;
          default:
            break;
        }
      }
      if (s_lower_bound > s_upper_bound) {
        const std::string msg = "s_lower_bound larger than s_upper_bound on STGraph";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      s_bounds_.emplace_back(s_lower_bound, s_upper_bound);
    }
  }

  speed_limit_ = st_graph_data.speed_limit();
  cruise_speed_ = reference_line_info_->GetCruiseSpeed();
  return Status::OK();
}

bool PiecewiseJerkSpeedNonlinearOptimizer::CheckSpeedLimitFeasibility() {
  // a naive check on first point of speed limit
  static constexpr double kEpsilon = 1e-6;
  const double init_speed_limit = speed_limit_.GetSpeedLimitByS(s_init_);
  if (init_speed_limit + kEpsilon < s_dot_init_) {
    AERROR << "speed limit [" << init_speed_limit
           << "] lower than initial speed[" << s_dot_init_ << "]";
    return false;
  }
  return true;
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SmoothSpeedLimit() {
  // using piecewise_jerk_path to fit a curve of speed_ref
  // TODO(Hongyi): move smooth configs to gflags
  double delta_s = 2.0;
  PrintCurves debug;
  std::vector<double> speed_ref;
  for (int i = 0; i < 100; ++i) {
    double path_s = i * delta_s;
    double limit = speed_limit_.GetSpeedLimitByS(path_s);
    speed_ref.emplace_back(limit);
    debug.AddPoint("coarse_v_bounds_upper", path_s, limit);
  }

  std::array<double, 3> init_state = {speed_ref[0], 0.0, 0.0};
  PiecewiseJerkPathProblem piecewise_jerk_problem(speed_ref.size(), 
                                                  delta_s,
                                                  init_state);
  piecewise_jerk_problem.set_x_bounds(0.0, 50.0);
  piecewise_jerk_problem.set_dx_bounds(-100.0, 100.0);
  piecewise_jerk_problem.set_ddx_bounds(-1000.0, 1000.0);
  piecewise_jerk_problem.set_dddx_bound(-10000.0, 100000.0);

  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(1.0);
  piecewise_jerk_problem.set_weight_ddx(1.0);
  piecewise_jerk_problem.set_weight_dddx(1.0);

  piecewise_jerk_problem.set_x_ref(10.0, std::move(speed_ref));

  if (!piecewise_jerk_problem.Optimize(4000)) {
    const std::string msg = "Smoothing speed limit failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Extract output
  std::vector<double> opt_x;
  std::vector<double> opt_dx;
  std::vector<double> opt_ddx;

  opt_x = piecewise_jerk_problem.opt_x();
  opt_dx = piecewise_jerk_problem.opt_dx();
  opt_ddx = piecewise_jerk_problem.opt_ddx();

  // Speed(s)
  PiecewiseJerkTrajectory1d smoothed_speed_limit(opt_x.front(), opt_dx.front(),
                                                 opt_ddx.front());

  for (size_t i = 1; i < opt_ddx.size(); ++i) {
    double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
    smoothed_speed_limit.AppendSegment(j, delta_s);
  }
  for (size_t i = 0; i < opt_x.size(); i++) {
    debug.AddPoint("smooth_v_bounds_upper", i * delta_s, opt_x[i]);
  }
  //   debug.PrintToLog();
  smoothed_speed_limit_ = smoothed_speed_limit;

  return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SmoothPathCurvature(const PathData& path_data) {
  // using piecewise_jerk_path to fit a curve of path kappa profile
  // TODO(Jinyun): move smooth configs to gflags
  const auto& cartesian_path = path_data.discretized_path();
  const double delta_s = 0.5;
  std::vector<double> path_curvature;
  PrintCurves debug;
  for (double path_s = cartesian_path.front().s(); path_s < cartesian_path.back().s() + delta_s; path_s += delta_s) {
    const auto& path_point = cartesian_path.Evaluate(path_s);
    path_curvature.push_back(path_point.kappa());
    debug.AddPoint("sk_corase", path_s, path_point.kappa());
  }

  const auto& path_init_point = cartesian_path.front();
  std::array<double, 3> init_state = {path_init_point.kappa(),
                                      path_init_point.dkappa(),
                                      path_init_point.ddkappa()};

  PiecewiseJerkPathProblem piecewise_jerk_problem(path_curvature.size(),
                                                  delta_s, init_state);
  piecewise_jerk_problem.set_x_bounds(-1.0, 1.0);
  piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(10.0);
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);

  piecewise_jerk_problem.set_x_ref(10.0, std::move(path_curvature));

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize(1000)) {
    const std::string msg = "Smoothing path curvature failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Extract output
  std::vector<double> opt_x;
  std::vector<double> opt_dx;
  std::vector<double> opt_ddx;
  opt_x = piecewise_jerk_problem.opt_x();
  opt_dx = piecewise_jerk_problem.opt_dx();
  opt_ddx = piecewise_jerk_problem.opt_ddx();

  // Kappa(s)
  PiecewiseJerkTrajectory1d smoothed_path_curvature(opt_x.front(), 
                                                    opt_dx.front(), 
                                                    opt_ddx.front()); // ? 只用到了第一个点？

  for (size_t i = 1; i < opt_ddx.size(); ++i) {
    double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
    smoothed_path_curvature.AppendSegment(j, delta_s);
  }

  for (size_t i = 0; i < opt_x.size(); ++i) {
    debug.AddPoint("sk_smooth", delta_s * i, opt_x[i]);
  }
  //   debug.PrintToLog();
  smoothed_path_curvature_ = smoothed_path_curvature; // ? 没有发现调用的地方

  return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByQP(SpeedData* const speed_data, std::vector<double>* distance,
                                                          std::vector<double>* velocity, std::vector<double>* acceleration) {
  std::array<double, 3> init_states = {s_init_, s_dot_init_, s_ddot_init_};

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, 
                                                   delta_t_,
                                                   init_states);
  piecewise_jerk_problem.set_dx_bounds(0.0, std::fmax(FLAGS_planning_upper_speed_limit, init_states[1]));
  piecewise_jerk_problem.set_ddx_bounds(s_ddot_min_, s_ddot_max_);
  piecewise_jerk_problem.set_dddx_bound(s_dddot_min_, s_dddot_max_);
  piecewise_jerk_problem.set_x_bounds(s_bounds_);

  // TODO(Jinyun): parameter tunnings
  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(0.0);
  piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());

  PrintCurves debug;
  std::vector<double> x_ref;
  for (int i = 0; i < num_of_knots_; ++i) {
    const double curr_t = i * delta_t_;       // 0.1s
    // get path_s
    SpeedPoint sp;
    speed_data->EvaluateByTime(curr_t, &sp);  // 参考速度dp结果
    x_ref.emplace_back(sp.s());
    debug.AddPoint("dp_st_curve", curr_t, sp.s());
  }
  piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    const std::string msg = "Speed Optimization by Quadratic Programming failed. st boundary is infeasible.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Extract output
  *distance = piecewise_jerk_problem.opt_x();
  *velocity = piecewise_jerk_problem.opt_dx();
  *acceleration = piecewise_jerk_problem.opt_ddx();
  for (size_t i = 0; i < distance->size(); i++) {
    const double curr_t = i * delta_t_;
    debug.AddPoint("warm_start_st", curr_t, distance->at(i));
    debug.AddPoint("warm_start_vt", curr_t, velocity->at(i));
    debug.AddPoint("warm_start_at", curr_t, acceleration->at(i));
  }
  //   debug.PrintToLog();
  return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByNLP(std::vector<double>* distance, std::vector<double>* velocity,
                                                           std::vector<double>* acceleration) {
  static std::mutex mutex_tnlp;
  UNIQUE_LOCK_MULTITHREAD(mutex_tnlp);

  // Set optimizer instance
  auto ptr_interface = new PiecewiseJerkSpeedNonlinearIpoptInterface(s_init_, s_dot_init_, s_ddot_init_, 
                                                                     delta_t_, num_of_knots_, total_length_,
                                                                     s_dot_max_, s_ddot_min_, s_ddot_max_,
                                                                     s_dddot_min_, s_dddot_max_);
      
  PrintCurves debug;
  ptr_interface->set_safety_bounds(s_bounds_);
  for (size_t i = 0; i < s_bounds_.size(); i++) {
    debug.AddPoint("st_bounds_lower", i * delta_t_, s_bounds_[i].first);
    debug.AddPoint("st_bounds_upper", i * delta_t_, s_bounds_[i].second);
  }

  ptr_interface->set_curvature_curve(smoothed_path_curvature_);
  ptr_interface->set_speed_limit_curve(smoothed_speed_limit_);

  // warm start : 使用速度QP作为初始解
  if (config_.use_warm_start()) {   // true
    const auto& warm_start_distance = *distance;
    const auto& warm_start_velocity = *velocity;
    const auto& warm_start_acceleration = *acceleration;
    if (warm_start_distance.empty() || warm_start_velocity.empty() || warm_start_acceleration.empty() ||
        warm_start_distance.size() != warm_start_velocity.size() ||
        warm_start_velocity.size() != warm_start_acceleration.size()) {
      const std::string msg = "Piecewise jerk speed nonlinear optimizer warm start invalid!";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    std::vector<std::vector<double>> warm_start;
    std::size_t size = warm_start_distance.size();
    for (std::size_t i = 0; i < size; ++i) {
      warm_start.emplace_back(std::initializer_list<double>({warm_start_distance[i], 
                                                             warm_start_velocity[i],
                                                             warm_start_acceleration[i]}));
    }
    ptr_interface->set_warm_start(warm_start);
  }

  // 惩罚速度优化结果接近dp引导线  true
  if (config_.use_smoothed_dp_guide_line()) {
    ptr_interface->set_reference_spatial_distance(*distance);
    ptr_interface->set_w_reference_spatial_distance(10.0);
  } else {
    std::vector<double> spatial_potantial(num_of_knots_, total_length_);
    ptr_interface->set_reference_spatial_distance(spatial_potantial);
    ptr_interface->set_w_reference_spatial_distance(config_.s_potential_weight());  // 0.05
  }

  // 使用边界软约束 true
  if (config_.use_soft_bound_in_nonlinear_speed_opt()) {
    ptr_interface->set_soft_safety_bounds(s_soft_bounds_);
    ptr_interface->set_w_soft_s_bound(config_.soft_s_bound_weight());
  }

  ptr_interface->set_w_overall_a(config_.acc_weight());
  ptr_interface->set_w_overall_j(config_.jerk_weight());
  ptr_interface->set_w_overall_centripetal_acc(config_.lat_acc_weight());

  ptr_interface->set_reference_speed(cruise_speed_);
  ptr_interface->set_w_reference_speed(config_.ref_v_weight());

  // 问题和求解器建立
  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 0);  // 关闭优化过程中日志输出
  app->Options()->SetIntegerValue("max_iter", 1000);

  // 求解器初始化
  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    const std::string msg = "Piecewise jerk speed nonlinear optimizer failed during initialization";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Solve the problem
  const auto start_timestamp = std::chrono::system_clock::now();
  status = app->OptimizeTNLP(problem);    // 该函数会根据提供的 problem 对象（包含优化问题的定义）执行优化

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "The optimization problem take time: " << diff.count() * 1000.0 << " ms.";

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    Ipopt::Number final_obj = app->Statistics()->FinalObjective();

    ADEBUG << "The problem solved in " << iter_count << " iterations!";
    ADEBUG << "The final value of the objective function is " << final_obj << '.';
  } else {
    const auto& ipopt_return_status = IpoptReturnStatus_Name(static_cast<IpoptReturnStatus>(status));
    if (ipopt_return_status.empty()) {
      AERROR << "Solver ends with unknown failure code: " << static_cast<int>(status);
    } else {
      AERROR << "Solver failure case is : " << ipopt_return_status;
    }
    const std::string msg = "Piecewise jerk speed nonlinear optimizer failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // extract primal results
  ptr_interface->get_optimization_results(distance, velocity, acceleration);
  for (size_t i = 0; i < distance->size(); i++) {
    debug.AddPoint("optimize_st_curve", i * delta_t_, (*distance)[i]);
    debug.AddPoint("optimize_vt_curve", i * delta_t_, (*velocity)[i]);
    debug.AddPoint("optimize_at_curve", i * delta_t_, (*acceleration)[i]);
    debug.AddPoint("optimize_sv_curve", (*distance)[i], (*velocity)[i]);
  }
  //   debug.PrintToLog();
  return Status::OK();
}
}  // namespace planning
}  // namespace apollo
