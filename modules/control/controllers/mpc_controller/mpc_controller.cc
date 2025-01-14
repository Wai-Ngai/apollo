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

#include "modules/control/controllers/mpc_controller/mpc_controller.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/mpc_osqp.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using Matrix = Eigen::MatrixXd;
using apollo::common::VehicleConfigHelper;

constexpr double GRA_ACC = 9.8;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);

  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, sizeof(name_buffer),
           "/tmp/mpc_controller_%F_%H%M%S.csv", &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {}
}  // namespace

MPCController::MPCController() : name_("MPC Controller") {
  if (FLAGS_enable_csv_debug) {
    mpc_log_file_.open(GetLogFileName());
    mpc_log_file_ << std::fixed;
    mpc_log_file_ << std::setprecision(6);
    WriteHeaders(mpc_log_file_);
  }
  AINFO << "Using " << name_;
}

MPCController::~MPCController() { CloseLogFile(); }

bool MPCController::LoadControlConf() {
  vehicle_param_ = VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf_.ts();
  if (ts_ <= 0.0) {
    AERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf_.cf();
  cr_ = control_conf_.cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ = vehicle_param_.max_steer_angle() * 180 / M_PI;
  max_lat_acc_ = control_conf_.max_lateral_acceleration();

  // TODO(Shu, Qi, Yu): add sanity check for conf values
  // steering ratio should be positive
  static constexpr double kEpsilon = 1e-6;
  if (std::isnan(steer_ratio_) || steer_ratio_ < kEpsilon) {
    AERROR << "[MPCController] steer_ratio = 0";
    return false;
  }
  wheel_single_direction_max_degree_ = steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
  max_acceleration_ = vehicle_param_.max_acceleration();
  max_deceleration_ = vehicle_param_.max_deceleration();

  const double mass_fl = control_conf_.mass_fl();
  const double mass_fr = control_conf_.mass_fr();
  const double mass_rl = control_conf_.mass_rl();
  const double mass_rr = control_conf_.mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  mpc_eps_ = control_conf_.eps();
  mpc_max_iteration_ = control_conf_.max_iteration();
  throttle_lowerbound_ = std::max(vehicle_param_.throttle_deadzone(),
                                  control_conf_.throttle_minimum_action());
  brake_lowerbound_ = std::max(vehicle_param_.brake_deadzone(),
                               control_conf_.brake_minimum_action());

  minimum_speed_protection_ = FLAGS_minimum_speed_protection;
  max_acceleration_when_stopped_ = FLAGS_max_acceleration_when_stopped;
  max_abs_speed_when_stopped_ = vehicle_param_.max_abs_speed_when_stopped();
  standstill_acceleration_ = control_conf_.standstill_acceleration();

  enable_mpc_feedforward_compensation_ =
      control_conf_.enable_mpc_feedforward_compensation();

  unconstrained_control_diff_limit_ =
      control_conf_.unconstrained_control_diff_limit();

  enable_look_ahead_back_control_ =
      control_conf_.enable_look_ahead_back_control();
  low_speed_bound_ = control_conf_.switch_speed();
  low_speed_window_ = control_conf_.switch_speed_window();
  preview_window_ = control_conf_.preview_window();
  lookahead_station_low_speed_ = control_conf_.lookahead_station();
  lookback_station_low_speed_ = control_conf_.lookback_station();
  lookahead_station_high_speed_ = control_conf_.lookahead_station_high_speed();
  lookback_station_high_speed_ = control_conf_.lookback_station_high_speed();
  use_lqr_curvature_feedforward_ =
      control_conf_.use_lqr_curvature_feedforward();

  enable_leadlag_ = control_conf_.enable_reverse_leadlag_compensation();
  if (enable_leadlag_) {
    leadlag_controller_.Init(control_conf_.reverse_leadlag_conf(), ts_);
  }

  preview_time_ = control_conf_.preview_window() * ts_;

  use_preview_ = control_conf_.use_preview();

  use_lookup_acc_pid_ = control_conf_.use_lookup_acc_pid();

  use_pitch_angle_filter_ = control_conf_.use_pitch_angle_filter();

  AINFO << "[MPCController] use_preview is " << use_preview_;
  InitControlCalibrationTable();
  ADEBUG << "MPC conf loaded";
  return true;
}

void MPCController::ProcessLogs(const SimpleMPCDebug *debug,
                                const canbus::Chassis *chassis) {
  // TODO(QiL): Add debug information
}

void MPCController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[MPCController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void MPCController::InitializeFilters() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts_, control_conf_.cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  common::LpfCoefficients(ts_,
                          control_conf_.pitch_angle_filter_conf().cutoff_freq(),
                          &den, &num);
  // 二阶数字滤波器，用于对方向盘角度控制指令滤波
  digital_filter_pitch_angle_.set_coefficients(den, num);
  // 均值低通滤波器，对横向误差CTE进行滤波
  lateral_error_filter_ = common::MeanFilter(
      static_cast<std::uint_fast8_t>(control_conf_.mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(
      static_cast<std::uint_fast8_t>(control_conf_.mean_filter_window_size()));
}

Status MPCController::Init(std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<MPCControllerConf>(&control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "failed to load mpc control_conf");
  }

  if (!ControlTask::LoadCalibrationTable(&calibration_table_)) {
    AERROR << "failed to load calibration table";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "mpc failed to load calibration table");
  }
  // 参数载入
  if (!LoadControlConf()) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // 车辆状态信息获取器
  injector_ = injector;
  // Matrix init operations.
  //给矩阵A中的常数项(与v无关的项)进行赋值，A矩阵中的非常数项在其他函数中每时刻更新 
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);    // 6*6
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;

  // TODO(QiL): expand the model to accommodate more combined states.
  //A矩阵中跟v相关的时变项，这些时变项分解成"常数项/v"的形式，然后提取出这个与v无关的常数项放在矩阵matrix_a_coeff_
  matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);// 6*6
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, controls_);               // 6*2
  matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(4, 1) = 0.0;
  matrix_b_(5, 1) = -1.0;
  matrix_bd_ = matrix_b_ * ts_;  // B*ts

  matrix_c_ = Matrix::Zero(basic_state_size_, 1);                        // 6*1
  matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

  matrix_state_ = Matrix::Zero(basic_state_size_, 1);                    // 6*1，系统状态
  matrix_k_ = Matrix::Zero(1, basic_state_size_);                        // 1*6

  matrix_r_ = Matrix::Identity(controls_, controls_);                    // 2*2

  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);        // 6*6

  int r_param_size = control_conf_.matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf_.matrix_r(i);
  }

  int q_param_size = control_conf_.matrix_q_size();
  if (basic_state_size_ != q_param_size) {
    const auto error_msg =
        absl::StrCat("MPC controller error: matrix_q size: ", q_param_size,
                     " in parameter file not equal to basic_state_size_: ",
                     basic_state_size_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_.matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  acceleration_lookup_pid_controller_.Init(control_conf_.acc_lookup_pid_conf());

  //用LatController类数据成员控制配置control_conf_去初始化滤波器
	//初始化3个滤波器：1个低通滤波是对计算出方向盘转角控制指令进行滤波
	//                2个滤波器是横向误差，航向误差的均值滤波器
  InitializeFilters();

  // 载入MPC增益表，就是在车速不同时，各个误差乘上个不同的比例
  // 车速越大，Q矩阵中横向误差的权重系数就越小
  // 车速越大，Q矩阵中航向误差的权重系数就越小，低速偏重于横向误差，高速偏重于航向误差
  LoadMPCGainScheduler();

  // 这个函数主要是在屏幕上打印一些车辆参数的信息
  LogInitParameters();

  ADEBUG << "[MPCController] init done!";
  return Status::OK();
}

void MPCController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && mpc_log_file_.is_open()) {
    mpc_log_file_.close();
  }
}

double MPCController::Wheel2SteerPct(const double wheel_angle) {
  return wheel_angle / wheel_single_direction_max_degree_ * 100;
}

void MPCController::Stop() { CloseLogFile(); }

std::string MPCController::Name() const { return name_; }

void MPCController::LoadMPCGainScheduler() {
  const auto &lat_err_gain_scheduler = control_conf_.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler = control_conf_.heading_err_gain_scheduler();
  const auto &feedforwardterm_gain_scheduler = control_conf_.feedforwardterm_gain_scheduler();
  const auto &steer_weight_gain_scheduler = control_conf_.steer_weight_gain_scheduler();

  ADEBUG << "MPC control gain scheduler loaded";

  // 定义了4张插值表 xy1, xy2, xy3, xy4，分别填充数据
  Interpolation1D::DataType xy1, xy2, xy3, xy4;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : feedforwardterm_gain_scheduler.scheduler()) {
    xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : steer_weight_gain_scheduler.scheduler()) {
    xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  //首先将MPCController类数据成员lat_err_interpolation_复位，然后用xy1去初始化lat_err_interpolation_
  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler for MPC controller";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler for MPC controller";

  feedforwardterm_interpolation_.reset(new Interpolation1D);
  ACHECK(feedforwardterm_interpolation_->Init(xy3))
      << "Fail to load feed forward term gain scheduler for MPC controller";

  steer_weight_interpolation_.reset(new Interpolation1D);
  ACHECK(steer_weight_interpolation_->Init(xy4))
      << "Fail to load steer weight gain scheduler for MPC controller";
}

Status MPCController::ComputeControlCommand(const localization::LocalizationEstimate *localization,
                                            const canbus::Chassis *chassis,
                                            const planning::ADCTrajectory *planning_published_trajectory,
                                            ControlCommand *cmd) {
  // 拷贝轨迹
  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(planning_published_trajectory));
  auto vehicle_state = injector_->vehicle_state();

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  if (((control_conf_.trajectory_transform_to_com_reverse() &&
        vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
       (control_conf_.trajectory_transform_to_com_reverse() &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE))) {
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }

  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = -control_conf_.cf();
    cr_ = -control_conf_.cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = control_conf_.cf();
    cr_ = control_conf_.cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  // 纵向误差计算
  if (!use_preview_) {
    ComputeLongitudinalErrors(&trajectory_analyzer_, preview_time_, ts_, debug);
  } else {
    ComputeLongitudinalErrors(&trajectory_analyzer_, debug);
  }

   // 横向误差计算，系统状态向量赋值
  UpdateState(debug);

  // 根据车速更新A,B,C矩阵
  UpdateMatrix(debug);

  // 转角前馈计算
  FeedforwardUpdate(debug);

  // Add gain scheduler for higher speed steering
  // 根据当前车速更新：横向误差惩罚系数，航向误差惩罚系数，前馈控制量，R矩阵（转角）
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) = matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    matrix_q_updated_(2, 2) = matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_ *
                                           feedforwardterm_interpolation_->Interpolate(vehicle_state->linear_velocity());
    matrix_r_updated_(0, 0) = matrix_r_(0, 0) * 
                              steer_weight_interpolation_->Interpolate(vehicle_state->linear_velocity());
  } else {
    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;
  }

  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));

  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));

  // 定义各种过程矩阵及滚动时域的矩阵序列
  Matrix control_matrix = Matrix::Zero(controls_, 1);                      // 2*1
  std::vector<Matrix> control(horizon_, control_matrix);                   // 10个2*1的矩阵

  Matrix control_gain_matrix = Matrix::Zero(controls_, basic_state_size_); // 2*6
  std::vector<Matrix> control_gain(horizon_, control_gain_matrix);         // 10个2*6的矩阵

  Matrix addition_gain_matrix = Matrix::Zero(controls_, 1);                // 2*1
  std::vector<Matrix> addition_gain(horizon_, addition_gain_matrix);       // 10个2*1的矩阵

  Matrix reference_state = Matrix::Zero(basic_state_size_, 1);             // 6*1  参考状态量，我们选取的模型是误差模型，所以当然希望最终的误差为零
  std::vector<Matrix> reference(horizon_, reference_state);                // 10个6*1的矩阵

  // 控制向量约束
  Matrix lower_bound(controls_, 1);                     // 2*1
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

  Matrix upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

  // 状态向量约束
  const double max = std::numeric_limits<double>::max();
  Matrix lower_state_bound(basic_state_size_, 1);        // 6*1
  Matrix upper_state_bound(basic_state_size_, 1);

  // lateral_error, lateral_error_rate, heading_error, heading_error_rate
  // station_error, station_error_rate
  lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
      -1.0 * max, -1.0 * max;
  upper_state_bound << max, max, M_PI, max, max, max;

  double mpc_start_timestamp = Clock::NowInSeconds();
  double steer_angle_feedback = 0.0;
  double acc_feedback = 0.0;
  double steer_angle_ff_compensation = 0.0;
  double unconstrained_control_diff = 0.0;
  double control_gain_truncation_ratio = 0.0;
  double unconstrained_control = 0.0;
  double steer_angle_feedback_augment = 0.0;
  const double v = injector_->vehicle_state()->linear_velocity();

  // 使用二次规划求解MPC
  std::vector<double> control_cmd(controls_, 0);  // 2个0

  apollo::common::math::MpcOsqp mpc_osqp(matrix_ad_, matrix_bd_, 
                                         matrix_q_updated_, matrix_r_updated_,
                                         matrix_state_,                          // 初始状态量
                                         lower_bound, upper_bound, 
                                         lower_state_bound, upper_state_bound, 
                                         reference_state,                        // 参考状态量
                                         mpc_max_iteration_, horizon_,
                                         mpc_eps_);
  if (!mpc_osqp.Solve(&control_cmd)) {
    AERROR << "MPC OSQP solver failed";
  } else {
    ADEBUG << "MPC OSQP problem solved! ";
    control[0](0, 0) = control_cmd.at(0);   // control是10个2*1的矩阵组成的vector，取vector的第一个元素（是一个矩阵），方向盘转角
    control[0](1, 0) = control_cmd.at(1);   // 加速度
  }

  steer_angle_feedback = Wheel2SteerPct(control[0](0, 0));
  acc_feedback = control[0](1, 0);
  for (int i = 0; i < basic_state_size_; ++i) {
    unconstrained_control += control_gain[0](0, i) * matrix_state_(i, 0);
  }
  unconstrained_control += addition_gain[0](0, 0) * v * debug->curvature();

  // 转向不足补偿计算 
  if (enable_mpc_feedforward_compensation_) {
    unconstrained_control_diff =
        Wheel2SteerPct(control[0](0, 0) - unconstrained_control);
    if (fabs(unconstrained_control_diff) <= unconstrained_control_diff_limit_) {
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v));
    } else {
      control_gain_truncation_ratio = control[0](0, 0) / unconstrained_control;
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v) *
                         control_gain_truncation_ratio);
    }
    if (std::isnan(steer_angle_ff_compensation)) {
      ADEBUG << "steer_angle_ff_compensation is nan";
      steer_angle_ff_compensation = 0.0;
    }
  } else {
    steer_angle_ff_compensation = 0.0;
  }

  double mpc_end_timestamp = Clock::NowInSeconds();

  ADEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.";

  if (enable_leadlag_) {
    if (control_conf_.enable_feedback_augment_on_high_speed() ||
        std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(vehicle_state->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = common::math::lerp(
            steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
            0.0, low_speed_bound_, std::fabs(vehicle_state->linear_velocity()));
      }
    }
  }

  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  // 转角命令为MPC反馈值 + 转角前馈值 + 补偿值（不清楚）
  double steer_angle =
      steer_angle_feedback + steer_angle_feedforwardterm_updated_ +
      steer_angle_ff_compensation + steer_angle_feedback_augment;

  // 转角命令限幅、低通滤波处理
  if (FLAGS_set_steer_limit) {
    const double steer_limit = std::atan(max_lat_acc_ * wheelbase_ /
                                         (vehicle_state->linear_velocity() *
                                          vehicle_state->linear_velocity())) *
                               steer_ratio_ * 180 / M_PI /
                               steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle with steer limitations at current speed
    double steer_angle_limited = common::math::Clamp(steer_angle, -steer_limit, steer_limit);   // 限幅
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited); // 滤波
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle_limited);
  }
  steer_angle = digital_filter_.Filter(steer_angle); // 滤波
  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);
  cmd->set_steering_target(steer_angle);

  debug->set_acceleration_cmd_closeloop(acc_feedback);

  double vehicle_pitch = 0.0;
  if (use_pitch_angle_filter_) {
    vehicle_pitch =
        digital_filter_pitch_angle_.Filter(injector_->vehicle_state()->pitch());
  } else {
    vehicle_pitch = injector_->vehicle_state()->pitch();
  }

  if (std::isnan(vehicle_pitch)) {
    AINFO << "pitch angle is nan.";
    vehicle_pitch = 0;
  }
  debug->set_vehicle_pitch(vehicle_pitch);

  double slope_offset_compensation = GRA_ACC * std::sin(vehicle_pitch);
  if (std::isnan(slope_offset_compensation)) {
    slope_offset_compensation = 0;
  }
  debug->set_slope_offset_compensation(slope_offset_compensation);

  // 加速度命令 = MPC加速度反馈 + 参考点加速度
  double acceleration_cmd = acc_feedback + debug->acceleration_reference() +
                            control_conf_.enable_slope_offset() * debug->slope_offset_compensation();
  // TODO(QiL): add pitch angle feed forward to accommodate for 3D control

  GetPathRemain(planning_published_trajectory, debug);
  // TODO(Yu): study the necessity of path_remain and add it to MPC if needed
  // At near-stop stage, replace the brake control command with the standstill
  // acceleration if the former is even softer than the latter
  if ((planning_published_trajectory->trajectory_type() ==
       apollo::planning::ADCTrajectory::NORMAL) ||
      (planning_published_trajectory->trajectory_type() ==
       apollo::planning::ADCTrajectory::SPEED_FALLBACK) ||
      (planning_published_trajectory->trajectory_type() ==
       apollo::planning::ADCTrajectory::UNKNOWN)) {
    if (control_conf_.use_preview_reference_check() &&
        (std::fabs(debug->preview_acceleration_reference()) <=
         FLAGS_max_acceleration_when_stopped) &&
        std::fabs(debug->preview_speed_reference()) <=
            vehicle_param_.max_abs_speed_when_stopped()) {
      debug->set_is_full_stop(true);
      ADEBUG << "Into full stop within preview acc and reference speed, "
             << "is_full_stop is " << debug->is_full_stop();
    }
    if (std::abs(debug->path_remain()) < FLAGS_max_acceleration_when_stopped) {
      debug->set_is_full_stop(true);
      ADEBUG << "Into full stop within path remain, "
             << "is_full_stop is " << debug->is_full_stop();
    }
  }

  if (debug->is_full_stop()) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd, standstill_acceleration_)
            : std::min(acceleration_cmd, standstill_acceleration_);
    Reset();
  }

  debug->set_acceleration_cmd(acceleration_cmd);

  acceleration_lookup_pid_controller_.SetPID(control_conf_.acc_lookup_pid_conf());

  double acceleration_lookup_error = acceleration_cmd - debug->acceleration_vrf();
  double acceleration_lookup_offset = 0.0;

  if (use_lookup_acc_pid_) {
    acceleration_lookup_offset = acceleration_lookup_pid_controller_.Control(acceleration_lookup_error, ts_);
  } else {
    acceleration_lookup_offset = 0.0;
  }

  double acceleration_lookup = acceleration_lookup_offset + acceleration_cmd;

  debug->set_acceleration_lookup_offset(acceleration_lookup_offset);

  debug->set_acceleration_lookup(acceleration_lookup);

  // 通过标定表获得车辆油门
  double calibration_value = 0.0;
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->preview_speed_reference(), acceleration_lookup));
  } else {
    calibration_value = control_interpolation_->Interpolate(std::make_pair(
        injector_->vehicle_state()->linear_velocity(), acceleration_lookup));
  }

  debug->set_calibration_value(calibration_value);

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (acceleration_cmd >= 0) {
    if (calibration_value >= 0) {
      throttle_cmd = std::max(calibration_value, throttle_lowerbound_);
    } else {
      throttle_cmd = throttle_lowerbound_;
    }
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound_;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound_);
    }
  }

  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  debug->set_heading(vehicle_state->heading());
  debug->set_steering_position(chassis->steering_percentage());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedforward_compensation(steer_angle_ff_compensation);
  debug->set_steer_unconstrained_control_diff(unconstrained_control_diff);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_position(chassis->steering_percentage());
  debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);

  if (std::fabs(vehicle_state->linear_velocity()) <= vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(planning_published_trajectory->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Status::OK();
}

void MPCController::InitControlCalibrationTable() {
  ADEBUG << "Control calibration table size is "
         << calibration_table_.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : calibration_table_.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  ACHECK(control_interpolation_->Init(xyz))
      << "Fail to init control calibration table";
}

void MPCController::UpdateState(SimpleMPCDebug *debug) {
  // 计算横向误差，放入debug指针
  const auto &com = injector_->vehicle_state()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(), injector_->vehicle_state()->heading(),
                       injector_->vehicle_state()->linear_velocity(),
                       injector_->vehicle_state()->angular_velocity(),
                       injector_->vehicle_state()->linear_acceleration(),
                       trajectory_analyzer_, debug);

  // State matrix update;
  // matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  // matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
  // State matrix update;
  // First four elements are fixed;
  if (enable_look_ahead_back_control_) {
    matrix_state_(0, 0) = debug->lateral_error_feedback();
    matrix_state_(2, 0) = debug->heading_error_feedback();
  } else {
    matrix_state_(0, 0) = debug->lateral_error();
    matrix_state_(2, 0) = debug->heading_error();
  }
}

void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  const double v = std::max(injector_->vehicle_state()->linear_velocity(),
                            minimum_speed_protection_);  // 0.1，防止除0
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols()); // 6*6
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *             // 离散化，A矩阵
               (matrix_i + ts_ * 0.5 * matrix_a_);

  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
  matrix_cd_ = matrix_c_ * debug->ref_heading_rate() * ts_;
}

void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  const double v = injector_->vehicle_state()->linear_velocity();
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  if (control_conf_.use_kinematic_model() &&
      injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    steer_angle_feedforwardterm_ =
        Wheel2SteerPct(wheelbase_ * debug->curvature());
  } else {
    steer_angle_feedforwardterm_ = Wheel2SteerPct(
        wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
  }
}

void MPCController::ComputeLateralErrors(const double x, const double y, 
                                         const double theta, const double linear_v,
                                         const double angular_v, const double linear_a,
                                         const TrajectoryAnalyzer &trajectory_analyzer, SimpleMPCDebug *debug) {
  // 计算匹配点，位置最近
  const auto matched_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  const double dx = x - matched_point.path_point().x();
  const double dy = y - matched_point.path_point().y();

  const double cos_matched_theta = std::cos(matched_point.path_point().theta());
  const double sin_matched_theta = std::sin(matched_point.path_point().theta());
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  double lateral_error = cos_matched_theta * dy - sin_matched_theta * dx;  // 横向误差：d
  if (control_conf_.enable_navigation_mode_error_filter()) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }
  debug->set_lateral_error(lateral_error);

  // matched_theta = matched_point.path_point().theta();
  debug->set_ref_heading(matched_point.path_point().theta());
  double delta_theta =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  if (control_conf_.enable_navigation_mode_error_filter()) {
    delta_theta = heading_error_filter_.Update(delta_theta);
  }
  debug->set_heading_error(delta_theta);                                      // 航向误差：

  if (enable_look_ahead_back_control_) {
    // Within the low-high speed transition window, linerly interplolate the
    // lookahead/lookback station for "soft" prediction window switch
    double lookahead_station = 0.0;
    double lookback_station = 0.0;
    if (std::fabs(linear_v) >= low_speed_bound_) {
      lookahead_station = lookahead_station_high_speed_;
      lookback_station = lookback_station_high_speed_;
    } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
      lookahead_station = lookahead_station_low_speed_;
      lookback_station = lookback_station_low_speed_;
    } else {
      lookahead_station = common::math::lerp(
          lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
          lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
      lookback_station = common::math::lerp(
          lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
          lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    }

    // Estimate the heading error with look-ahead/look-back windows as feedback
    // signal for special driving scenarios
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      heading_error_feedback = delta_theta;
    } else {
      auto lookahead_point =
          trajectory_analyzer.QueryNearestPointByRelativeTime(
              matched_point.relative_time() +
              lookahead_station /
                  (std::max(std::fabs(linear_v), 0.1) * std::cos(delta_theta)));
      heading_error_feedback = common::math::NormalizeAngle(
          delta_theta + matched_point.path_point().theta() -
          lookahead_point.path_point().theta());
    }
    debug->set_heading_error_feedback(heading_error_feedback);
    // Estimate the lateral error with look-ahead/look-back windows as feedback
    // signal for special driving scenarios
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_feedback =
          lateral_error - lookback_station * std::sin(delta_theta);
    } else {
      lateral_error_feedback =
          lateral_error + lookahead_station * std::sin(delta_theta);
    }
    debug->set_lateral_error_feedback(lateral_error_feedback);
  }

  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  double lateral_error_dot = linear_v * sin_delta_theta;                      // 横向误差变化率
  double lateral_error_dot_dot = linear_a * sin_delta_theta;
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }

  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point().kappa());
  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  // debug->set_heading_rate(angular_v);
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(debug->curvature() * matched_point.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
}

void MPCController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer,
                                              SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  // 寻找位置最近匹配点
  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y(),
      injector_->vehicle_state()->heading(),
      injector_->vehicle_state()->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  const double current_control_time = Clock::NowInSeconds();

  // 寻找时间最近参考点
  TrajectoryPoint reference_point =trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();

  const double linear_v = injector_->vehicle_state()->linear_velocity();
  const double linear_a = injector_->vehicle_state()->linear_acceleration();
  double heading_error = common::math::NormalizeAngle(injector_->vehicle_state()->heading() - matched_point.theta());
  double lon_speed = linear_v * std::cos(heading_error);
  double lon_acceleration = linear_a * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *  // 横向误差系数
                                             linear_v * std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_station_feedback(s_matched);  // current station
  debug->set_station_error(reference_point.path_point().s() - s_matched);      // 位置误差：s_ref - s_match
  debug->set_speed_reference(reference_point.v());
  debug->set_speed_feedback(lon_speed);  // current speed
  debug->set_speed_error(reference_point.v() - s_dot_matched);                 // 速度误差：v_ref - s_match_dot
  debug->set_acceleration_reference(reference_point.a());
  debug->set_acceleration_feedback(lon_acceleration);
  debug->set_acceleration_error(reference_point.a() -                         // 加速度误差：
                                lon_acceleration / one_minus_kappa_lat_error);

  double jerk_reference = (debug->acceleration_reference() - previous_acceleration_reference_) / ts_;
  double lon_jerk = (debug->acceleration_feedback() - previous_acceleration_) / ts_;
  debug->set_jerk_reference(jerk_reference);
  debug->set_jerk_feedback(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->acceleration_feedback();
}

void MPCController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer, 
                                              const double preview_time,
                                              const double ts, SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  auto vehicle_state = injector_->vehicle_state();
  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(vehicle_state->x(), vehicle_state->y());

  trajectory_analyzer->ToTrajectoryFrame(vehicle_state->x(), vehicle_state->y(), 
                                         vehicle_state->heading(),
                                         vehicle_state->linear_velocity(), matched_point, &s_matched,
                                         &s_dot_matched, &d_matched, &d_dot_matched);

  // double current_control_time = Time::Now().ToSecond();
  double current_control_time = ::apollo::cyber::Clock::NowInSeconds();
  double preview_control_time = current_control_time + preview_time;

  TrajectoryPoint reference_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time);
  TrajectoryPoint preview_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(preview_control_time);

  debug->mutable_current_matched_point()->mutable_path_point()->set_x(matched_point.x());
  debug->mutable_current_matched_point()->mutable_path_point()->set_y(matched_point.y());
  debug->mutable_current_reference_point()->mutable_path_point()->set_x(reference_point.path_point().x());
  debug->mutable_current_reference_point()->mutable_path_point()->set_y(reference_point.path_point().y());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_x(preview_point.path_point().x());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_y(preview_point.path_point().y());

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  ADEBUG << "preview point:" << preview_point.DebugString();

  double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                      matched_point.theta());
  double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
  double lon_acceleration = vehicle_state->linear_acceleration() * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_station_feedback(s_matched);
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_reference(reference_point.v());
  debug->set_speed_feedback(lon_speed);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_acceleration_feedback(lon_acceleration);
  debug->set_acceleration_vrf(vehicle_state->linear_acceleration());
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference = (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk = (debug->acceleration_feedback() - previous_acceleration_) / ts;
  debug->set_jerk_reference(jerk_reference);
  debug->set_jerk_feedback(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->acceleration_feedback();
  previous_acceleration_vrf_ = debug->acceleration_vrf();

  debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
  debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
}

// TODO(all): Refactor and simplify
void MPCController::GetPathRemain(const planning::ADCTrajectory *planning_published_trajectory,
                                  SimpleMPCDebug *debug) {
  int stop_index = 0;
  static constexpr double kSpeedThreshold = 1e-3;
  static constexpr double kForwardAccThreshold = -1e-2;
  static constexpr double kBackwardAccThreshold = 1e-1;
  static constexpr double kParkingSpeed = 0.1;

  if (planning_published_trajectory->gear() == canbus::Chassis::GEAR_DRIVE) {
    while (stop_index < planning_published_trajectory->trajectory_point_size()) {
      auto &current_trajectory_point = planning_published_trajectory->trajectory_point(stop_index);
      if (fabs(current_trajectory_point.v()) < kSpeedThreshold &&
          current_trajectory_point.a() > kForwardAccThreshold &&
          current_trajectory_point.a() < 0.0) {
        break;
      }
      ++stop_index;
    }
  } else {
    while (stop_index < planning_published_trajectory->trajectory_point_size()) {
      auto &current_trajectory_point = planning_published_trajectory->trajectory_point(stop_index);
      if (current_trajectory_point.v() > -kSpeedThreshold &&
          current_trajectory_point.a() < kBackwardAccThreshold &&
          current_trajectory_point.a() > 0.0) {
        break;
      }
      ++stop_index;
    }
  }
  ADEBUG << "stop_index is, " << stop_index;
  if (stop_index == planning_published_trajectory->trajectory_point_size()) {
    --stop_index;
    if (fabs(planning_published_trajectory->trajectory_point(stop_index).v()) < kParkingSpeed) {
      ADEBUG << "the last point is selected as parking point";
    } else {
      ADEBUG << "the last point found in path and speed > speed_deadzone";
    }
  }
  debug->set_path_remain(planning_published_trajectory->trajectory_point(stop_index).path_point().s() -
                         debug->station_feedback());
}

}  // namespace control
}  // namespace apollo
