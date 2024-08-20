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

#include "modules/control/controllers/lat_based_lqr_controller/lat_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using apollo::cyber::Clock;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

LatController::LatController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

LatController::~LatController() { CloseLogFile(); }

bool LatController::LoadControlConf() {
  //LatController类内成员车辆参数配置vehicle_param_加载车辆参数配置 反序列化配置文件
  vehicle_param_ = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = lat_based_lqr_controller_conf_.ts();
  if (ts_ <= 0.0) {
    AERROR << "[LatController] Invalid control update interval.";
    return false;
  }
  cf_ = lat_based_lqr_controller_conf_.cf();
  cr_ = lat_based_lqr_controller_conf_.cr();
  preview_window_ = lat_based_lqr_controller_conf_.preview_window();
  lookahead_station_low_speed_ = lat_based_lqr_controller_conf_.lookahead_station(); // 低速前行预瞄距离，修复不同速度下参考点的对准问题
  lookback_station_low_speed_ = lat_based_lqr_controller_conf_.lookback_station();
  lookahead_station_high_speed_ = lat_based_lqr_controller_conf_.lookahead_station_high_speed();
  lookback_station_high_speed_ = lat_based_lqr_controller_conf_.lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ = vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = lat_based_lqr_controller_conf_.max_lateral_acceleration();
  low_speed_bound_ = lat_based_lqr_controller_conf_.switch_speed();
  low_speed_window_ = lat_based_lqr_controller_conf_.switch_speed_window();

  const double mass_fl = lat_based_lqr_controller_conf_.mass_fl();
  const double mass_fr = lat_based_lqr_controller_conf_.mass_fr();
  const double mass_rl = lat_based_lqr_controller_conf_.mass_rl();
  const double mass_rr = lat_based_lqr_controller_conf_.mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia  车辆绕z轴的转动惯量(又叫惯性矩) m*r^2
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = lat_based_lqr_controller_conf_.eps();                     // LQR优化的最小精度0.01，停止条件1
  lqr_max_iteration_ = lat_based_lqr_controller_conf_.max_iteration(); // LQR最大迭代次数，停止条件2

  query_relative_time_ = lat_based_lqr_controller_conf_.query_relative_time();          // 相对时间  机构延时，一般来说发出指令到底盘响应会有500ms延时

  minimum_speed_protection_ = FLAGS_minimum_speed_protection;

  return true;
}

void LatController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  const std::string log_str = absl::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(), ",",
      debug->heading_error(), ",", debug->heading_error_rate(), ",",
      debug->lateral_error_rate(), ",", debug->curvature(), ",",
      debug->steer_angle(), ",", debug->steer_angle_feedforward(), ",",
      debug->steer_angle_lateral_contribution(), ",",
      debug->steer_angle_lateral_rate_contribution(), ",",
      debug->steer_angle_heading_contribution(), ",",
      debug->steer_angle_heading_rate_contribution(), ",",
      debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
      injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void LatController::InitializeFilters() {
  // Low pass filter  去除高频噪声
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts_, lat_based_lqr_controller_conf_.cutoff_freq(),
                          &den, &num);
  digital_filter_.set_coefficients(den, num);   //用于对方向盘转角控制指令进行滤波
  
  // mean filter 防止突变
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(lat_based_lqr_controller_conf_.mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(lat_based_lqr_controller_conf_.mean_filter_window_size()));
}

Status LatController::Init(std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<LatBaseLqrControllerConf>(&lat_based_lqr_controller_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "failed to load lat control_conf");
  }
  injector_ = injector;
  if (!LoadControlConf()) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;      // 矩阵大小，默认4
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  /*
  A matrix (Gear Drive) 前进档的车辆状态方程系数矩阵A，系数矩阵A是随着每时刻速度变化而变化的线性时变矩阵 
  [0.0, 1.0,                             0.0,                           0.0;
   0.0, (-(c_f + c_r) / m) / v,          (c_f + c_r) / m,               (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0,                             0.0,                           1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  //给矩阵A中的常数项(与v无关的项)进行赋值，A矩阵中的非常数项在其他函数中每时刻更新
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  //A矩阵中跟v相关的时变项，这些时变项分解成"常数项/v"的形式，然后提取出这个与v无关的常数项放在矩阵matrix_a_coeff_
  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;   //矩阵B的离散形式Bd就等于 B * ts

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);                       // R（1*1），控制量作用权重
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);       // R（4*4），状态量权重

  int q_param_size = lat_based_lqr_controller_conf_.matrix_q_size();
  int reverse_q_param_size = lat_based_lqr_controller_conf_.reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    const auto error_msg = absl::StrCat("lateral controller error: matrix_q size: ", q_param_size,
                                        "lateral controller error: reverse_matrix_q size: ", reverse_q_param_size,
                                        " in parameter file not equal to matrix_size: ", matrix_size);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  //加载控制配置中matrix_q(0),matrix_q(1),matrix_q(2),matrix_q(3)。默认分别为0.05，0.0，1.0，0.0
  //可以看出实际上只考虑了横向误差和航向误差且航向误差的比重要比横向误差大很多，误差变化率Q阵中系数为0
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = lat_based_lqr_controller_conf_.matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;

  //用LatController类数据成员控制配置control_conf_去初始化滤波器
	//初始化3个滤波器：1个低通滤波是对计算出方向盘转角控制指令进行滤波
	//                2个滤波器是横向误差，航向误差的均值滤波器
  InitializeFilters();

  //LoadLatGainScheduler加载增益调度表，就是横向误差和航向误差在车速不同时乘上个不同的比例
	//这个比例决定了实际时的控制效果，根据实际经验低速和高速应该采取不同的比例，低速比例较大，若高速
	//采用同样比例极有可能导致画龙现象，问题来了，对于一辆给定的车，增益调度表如何确定？
  LoadLatGainScheduler();

  //这个函数主要是在屏幕上打印一些车辆参数的信息
  LogInitParameters();

  //默认是开启横向控制中的超前滞后控制器的，提升或者降低闭环反馈系统的响应速度
  enable_leadlag_ = lat_based_lqr_controller_conf_.enable_reverse_leadlag_compensation();
  if (enable_leadlag_) {
    leadlag_controller_.Init(lat_based_lqr_controller_conf_.reverse_leadlag_conf(), ts_);
  }

  //默认关闭mrac模型参考自适应控制
  enable_mrac_ = lat_based_lqr_controller_conf_.enable_steer_mrac_control();
  if (enable_mrac_) {
    mrac_controller_.Init(lat_based_lqr_controller_conf_.steer_mrac_conf(),
                          vehicle_param_.steering_latency_param(), ts_);
  }

  //默认打开前进倒车时的预瞄控制
  enable_look_ahead_back_control_ = lat_based_lqr_controller_conf_.enable_look_ahead_back_control();

  return Status::OK();
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatController::LoadLatGainScheduler() {
  const auto &lat_err_gain_scheduler = lat_based_lqr_controller_conf_.lat_err_gain_scheduler();          // 不同速度下的横向偏差增益
  const auto &heading_err_gain_scheduler = lat_based_lqr_controller_conf_.heading_err_gain_scheduler();  // 不同速度下的航向偏差增益
  AINFO << "Lateral control gain scheduler loaded";
  
  //定义了两张插值表xy1, xy2，分别填充数据
  Interpolation1D::DataType xy1, xy2;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  //首先将LatController类数据成员lat_err_interpolation_复位，然后用xy1去初始化lat_err_interpolation_
  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

Status LatController::ComputeControlCommand(const localization::LocalizationEstimate *localization,
                                            const canbus::Chassis *chassis,
                                            const planning::ADCTrajectory *planning_published_trajectory,
                                            ControlCommand *cmd) {
  auto vehicle_state = injector_->vehicle_state();
  auto previous_lon_debug = injector_->Get_previous_lon_debug_info();
  auto target_tracking_trajectory = *planning_published_trajectory;

  if (FLAGS_use_navigation_mode &&
      lat_based_lqr_controller_conf_.enable_navigation_mode_position_update()) {
    auto time_stamp_diff = planning_published_trajectory->header().timestamp_sec() -
                           current_trajectory_timestamp_;

    auto curr_vehicle_x = localization->pose().position().x();
    auto curr_vehicle_y = localization->pose().position().y();

    double curr_vehicle_heading = 0.0;
    const auto &orientation = localization->pose().orientation();
    if (localization->pose().has_heading()) {
      curr_vehicle_heading = localization->pose().heading();
    } else {
      curr_vehicle_heading = common::math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                                               orientation.qy(), orientation.qz());
    }

    // new planning trajectory
    if (time_stamp_diff > 1.0e-6) {
      init_vehicle_x_ = curr_vehicle_x;
      init_vehicle_y_ = curr_vehicle_y;
      init_vehicle_heading_ = curr_vehicle_heading;

      current_trajectory_timestamp_ = planning_published_trajectory->header().timestamp_sec();
    } else {
      auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
      auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
      auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

      auto cos_map_veh = std::cos(init_vehicle_heading_);
      auto sin_map_veh = std::sin(init_vehicle_heading_);

      auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
      auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

      auto cos_theta_diff = std::cos(-theta_diff);
      auto sin_theta_diff = std::sin(-theta_diff);

      auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
      auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

      auto ptr_trajectory_points = target_tracking_trajectory.mutable_trajectory_point();

      std::for_each(ptr_trajectory_points->begin(), 
                    ptr_trajectory_points->end(),
                    [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
                     &theta_diff](common::TrajectoryPoint &p) {
                      auto x = p.path_point().x();
                      auto y = p.path_point().y();
                      auto theta = p.path_point().theta();
          
                      auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
                      auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
                      auto theta_new = common::math::NormalizeAngle(theta - theta_diff);
          
                      p.mutable_path_point()->set_x(x_new);
                      p.mutable_path_point()->set_y(y_new);
                      p.mutable_path_point()->set_theta(theta_new);
                    });
    }
  }

  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  // 低速时使用几何中心点 ，倒车的时候使用后轴的中心，而在前进的时候使用车辆的几何中心
  if (((lat_based_lqr_controller_conf_.trajectory_transform_to_com_reverse() && vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
       (lat_based_lqr_controller_conf_.trajectory_transform_to_com_drive() && vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)) &&
      enable_look_ahead_back_control_) {
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }

  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)
  if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0,                            1.0 * v                         0.0;
     0.0, (-(c_f + c_r) / m) / v,          (c_f + c_r) / m,               (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0,                             0.0,                           1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // 因为R档和前面加载LoadControlConf()函数里加载的符号不同需要更新,下面4项都是D档，R档A中会变化的项，D档和R档这4项不同
    cf_ = -lat_based_lqr_controller_conf_.cf();
    cr_ = -lat_based_lqr_controller_conf_.cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0,                             0.0,                           0.0;
     0.0, (-(c_f + c_r) / m) / v,          (c_f + c_r) / m,               (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0,                             0.0,                           1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    //下面4项都是D档，R档A中会变化的项，D档和R档这4项不同
    cf_ = lat_based_lqr_controller_conf_.cf();
    cr_ = lat_based_lqr_controller_conf_.cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  //万一D档和R档切换，A矩阵的值是在变化的话，所以D档和R档时要按照各自的方式计算一下，因为cf_,cr_也更新了
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  //倒挡更新航向角，也是要满足FLAGS_reverse_heading_control默认关闭
  UpdateDrivingOrientation();

	//SimpleLateralDebug类由modules/control/proto/control_cmd.proto文件生成
	//从名字就可以看出SimpleLateralDebug,"简单横向调试"，这个类就是用来调试横向控制的，里面用来储存一些横向控制过程量
  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  //更新车辆状态矩阵X=[e1 e1' e2 e2']
	//首先该函数UpdateState()内部调用了ComputeLateralErrors()函数得到的各种误差信息存放到debug中
	//然后又用debug去更新车辆状态矩阵X即matrix_state_
  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug, chassis);

  //主要是更新车辆状态方程系数矩阵A及其离散形式中与速度相关的时变项
  UpdateMatrix();

  // Compound discrete matrix with road preview model
  UpdateMatrixCompound();

  // Adjust matrix_q_updated when in reverse gear
  int q_param_size = lat_based_lqr_controller_conf_.matrix_q_size();                   // 读取Q矩阵参数大小
  int reverse_q_param_size = lat_based_lqr_controller_conf_.reverse_matrix_q_size();
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {                 // 赋值Q矩阵
    //R档加载控制配置里的reverse_matrix_q
    for (int i = 0; i < reverse_q_param_size; ++i) {
      matrix_q_(i, i) = lat_based_lqr_controller_conf_.reverse_matrix_q(i);
    }
  } else {
    //非R档加载控制配置里的matrix_q
    for (int i = 0; i < q_param_size; ++i) {
      matrix_q_(i, i) = lat_based_lqr_controller_conf_.matrix_q(i);
    }
  }

  // Add gain scheduler for higher speed steering
  if (FLAGS_enable_gain_scheduler) {                                                      // 根据车速对横向偏差和航向偏差增益进行插值
    //Q(1,1)=Q(1,1)*(用之前加载的横向误差调度增益表根据当前车速插值得到的ratio)
    matrix_q_updated_(0, 0) = matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(      // 高速时对应降低相关权重  避免猛打方向
                              std::fabs(vehicle_state->linear_velocity()));
    matrix_q_updated_(2, 2) = matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              std::fabs(vehicle_state->linear_velocity()));
    
    //求解LQR问题，求解到的最优状态反馈矩阵K放入matrix_k_中
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,              // LQR求解器
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,                  // 输入：A B Q R eps iteration
                                  &matrix_k_);                                              // 输出：矩阵K  1*4  状态量权重系数
  } else {
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  }

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /  // 侧偏刚度单位  N/弧度
                                      M_PI * steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;

  //调用函数ComputeFeedForward计算前馈控制量
  const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());

  double steer_angle = 0.0;
  double steer_angle_feedback_augment = 0.0;
  // Augment the feedback control on lateral error at the desired speed domain
  // 如果打开leadlag超前滞后控制器
  if (enable_leadlag_) {
    //如果车辆打开高速的反馈增强控制 或 车速小于低高速边界速度
    if (lat_based_lqr_controller_conf_.enable_feedback_augment_on_high_speed() ||
        std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment = leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(vehicle_state->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = common::math::lerp(steer_angle_feedback_augment, 
                                                          low_speed_bound_ - low_speed_window_,
                                                          0.0, low_speed_bound_, 
                                                          std::fabs(vehicle_state->linear_velocity()));
      }
    }
  }
  //总的方向盘转角控制量 = 反馈控制量 + 前馈控制量 + 增强反馈控制量(超前滞后控制器)
  steer_angle = steer_angle_feedback + steer_angle_feedforward + steer_angle_feedback_augment;

  // Compute the steering command limit with the given maximum lateral acceleration
  //若限制横向加速度 最大方向盘转角百分数 = atan(ay_max * L / v^2) * steerratio * 180/pi /max_steer_ang * 100
  const double steer_limit = FLAGS_set_steer_limit 
                             ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (vehicle_state->linear_velocity() * vehicle_state->linear_velocity())) *
                               steer_ratio_ * 180 / M_PI / steer_single_direction_max_degree_ * 100
                            : 100.0;

  //对方向盘转动速率进行限制 一个周期方向盘转角最大增量 = 最大方向盘角速度 * 控制周期
  //此刻方向盘转角控制量只能在范围内：上一时刻方向盘转角控制量 +/- 一个周期方向盘转角最大增量
  const double steer_diff_with_max_rate = lat_based_lqr_controller_conf_.enable_maximum_steer_rate_limit()
                                          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI / steer_single_direction_max_degree_ * 100
                                          : 100.0;

  //方向盘实际转角
  const double steering_position = chassis->steering_percentage();

  // Re-compute the steering command if the MRAC control is enabled, with steer
  // angle limitation and steer rate limitation
  if (enable_mrac_) {
    const int mrac_model_order = lat_based_lqr_controller_conf_.steer_mrac_conf().mrac_model_order();
    Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
    steer_state(0, 0) = chassis->steering_percentage();
    if (mrac_model_order > 1) {
      steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
    }
    if (std::fabs(vehicle_state->linear_velocity()) > FLAGS_minimum_speed_resolution) {
      mrac_controller_.SetStateAdaptionRate(1.0);
      mrac_controller_.SetInputAdaptionRate(1.0);
    } else {
      mrac_controller_.SetStateAdaptionRate(0.0);
      mrac_controller_.SetInputAdaptionRate(0.0);
    }
    steer_angle = mrac_controller_.Control(steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
    // Set the steer mrac debug message
    MracDebug *mracdebug = debug->mutable_steer_mrac_debug();
    Matrix steer_reference = mrac_controller_.CurrentReferenceState();
    mracdebug->set_mrac_model_order(mrac_model_order);
    for (int i = 0; i < mrac_model_order; ++i) {
      mracdebug->add_mrac_reference_state(steer_reference(i, 0));
      mracdebug->add_mrac_state_error(steer_state(i, 0) -
                                      steer_reference(i, 0));
      mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(mrac_controller_.CurrentStateAdaptionGain()(i, 0));
    }
    mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(mrac_controller_.CurrentInputAdaptionGain()(0, 0));
    mracdebug->set_mrac_reference_saturation_status(mrac_controller_.ReferenceSaturationStatus());
    mracdebug->set_mrac_control_saturation_status(mrac_controller_.ControlSaturationStatus());
  }
  pre_steering_position_ = steering_position;
  debug->set_steer_mrac_enable_status(enable_mrac_);

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited = common::math::Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  // 对方向盘转角数字滤波，然后控制百分数又限制在正负100
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  // Check if the steer is locked and hence the previous steer angle should be executed
  if (injector_->vehicle_state()->gear() != canbus::Chassis::GEAR_REVERSE) {
    if ((std::abs(vehicle_state->linear_velocity()) < lat_based_lqr_controller_conf_.lock_steer_speed() ||
         previous_lon_debug->path_remain() <= 0) &&
        vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE &&
        chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
      ADEBUG << "Into lock steer, path_remain is "
             << previous_lon_debug->path_remain() << "linear_velocity is "
             << vehicle_state->linear_velocity();
      steer_angle = pre_steer_angle_;
    }
  }

  // Set the steer commands
  // 设定转角指令，再通过最大转角速率再次进行限制幅度，最多只能=上次的转角指令+/-最大转角速率 * Ts
  cmd->set_steering_target(common::math::Clamp(steer_angle, 
                                               pre_steer_angle_ - steer_diff_with_max_rate,
                                               pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();

  // compute extra information for logging and debugging
  // -k1*e1 e1为横向误差 这一项就是横向误差贡献的控制量百分数
  const double steer_angle_lateral_contribution = -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_ /
                                                  steer_single_direction_max_degree_ * 100;

  //横向误差率贡献的控制量百分数
  const double steer_angle_lateral_rate_contribution = -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_ /
                                                       steer_single_direction_max_degree_ * 100;
  //航向误差贡献的控制量百分数
  const double steer_angle_heading_contribution = -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_ /
                                                  steer_single_direction_max_degree_ * 100;
  //航向误差率贡献的控制量百分数
  const double steer_angle_heading_rate_contribution = -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_ /
                                                       steer_single_direction_max_degree_ * 100;

  debug->set_heading(driving_orientation_);
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
  debug->set_steering_position(steering_position);
  debug->set_ref_speed(vehicle_state->linear_velocity());

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status LatController::Reset() {
  matrix_state_.setZero();
  if (enable_mrac_) {
    mrac_controller_.Reset();
  }
  return Status::OK();
}

void LatController::UpdateState(SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  auto vehicle_state = injector_->vehicle_state();
  if (FLAGS_use_navigation_mode) {
    ComputeLateralErrors(0.0, 0.0, driving_orientation_, 
                         vehicle_state->linear_velocity(),
                         vehicle_state->angular_velocity(), 
                         vehicle_state->linear_acceleration(),
                         trajectory_analyzer_, debug, chassis);
  } else {
    // Transform the coordinate of the vehicle states from the center of the
    // rear-axis to the center of mass, if conditions matched
    const auto &com = vehicle_state->ComputeCOMPosition(lr_);
    ComputeLateralErrors(com.x(), com.y(), driving_orientation_,
                         vehicle_state->linear_velocity(),
                         vehicle_state->angular_velocity(),
                         vehicle_state->linear_acceleration(),
                         trajectory_analyzer_, debug, chassis);
  }

  // State matrix update;
  // First four elements are fixed;
  if (enable_look_ahead_back_control_) {
      //当打开这个e1和e3分别赋值横向反馈误差和航向反馈误差
    //若打开lookahead（D档）,lookback(R档)则x中的e1,e3就为考虑了lookahead的误差
	  //lateral_error_feedback = lateral_error + 参考点到lookahead点的横向误差
	  //heading_error_feedback = heading_error + ref_heading - lookahead点的heading 实际上就是匹配点到lookahead点的航向差 
    matrix_state_(0, 0) = debug->lateral_error_feedback();
    matrix_state_(2, 0) = debug->heading_error_feedback();
  } else {
    matrix_state_(0, 0) = debug->lateral_error();
    matrix_state_(2, 0) = debug->heading_error();   //heading_error = 车辆heading - ref_heading
  }
  matrix_state_(1, 0) = debug->lateral_error_rate(); //把debug中的横向误差率和航向误差率更新进状态矩阵中
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  // 这一部分是更新状态矩阵中的preview项
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);

    const auto preview_point = trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);
    const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(preview_point.path_point().x(), 
                                                                                preview_point.path_point().y());

    const double dx = preview_point.path_point().x() - matched_point.path_point().x();
    const double dy = preview_point.path_point().y() - matched_point.path_point().y();

    const double cos_matched_theta = std::cos(matched_point.path_point().theta());
    const double sin_matched_theta = std::sin(matched_point.path_point().theta());
    const double preview_d_error = cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

void LatController::UpdateMatrix() {
  double v;
  // At reverse driving, replace the lateral translational motion dynamics with
  // the corresponding kinematic models
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE &&
      !lat_based_lqr_controller_conf_.reverse_use_dynamic_model()) {
    v = std::min(injector_->vehicle_state()->linear_velocity(),
                 -minimum_speed_protection_);
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    v = std::max(injector_->vehicle_state()->linear_velocity(),
                 minimum_speed_protection_);
    matrix_a_(0, 2) = 0.0;  //非R档A矩阵的1行3列为0
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *  //计算A矩阵的离散化形式Ad，用双线性变换法
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LatController::ComputeFeedForward(double ref_curvature) const {
  const double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = injector_->vehicle_state()->linear_velocity();
  double steer_angle_feedforwardterm;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE &&
      !lat_based_lqr_controller_conf_.reverse_use_dynamic_model()) {
    steer_angle_feedforwardterm = lat_based_lqr_controller_conf_.reverse_feedforward_ratio() *
                                  wheelbase_ * ref_curvature * 180 / M_PI * steer_ratio_ /
                                  steer_single_direction_max_degree_ * 100;
  } else {
    steer_angle_feedforwardterm = (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
                                   matrix_k_(0, 2) * (lr_ * ref_curvature -
                                                      lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
                                  180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
  }

  return steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(const double x, const double y, const double theta, 
                                         const double linear_v, const double angular_v, const double linear_a,
                                         const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug,
                                         const canbus::Chassis *chassis) {
  TrajectoryPoint target_point;

//始终将车辆当前时间向前加0.8秒在轨迹上对应的点作为目标点
  if (lat_based_lqr_controller_conf_.query_time_nearest_point_only()) {
    //如果是，只将车辆当前的时间，向前加固定时间长度后在轨迹上对应点作为目标点     // 查找时间匹配点
    target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(Clock::NowInSeconds() + query_relative_time_);
     //query_relative_time_ = control_conf->query_relative_time();这个值是从控制配置文件中读取，默认0.8秒
  } else {
    if (FLAGS_use_navigation_mode &&
        !lat_based_lqr_controller_conf_.enable_navigation_mode_position_update()) {
      target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(Clock::NowInSeconds() + query_relative_time_);
    } else {
      // 目标点取轨迹上距离当前车辆xy坐标点最近的点，默认目标点就是取距离最近点
      target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);   // 查找距离匹配点
    }
  }
  const double dx = x - target_point.path_point().x(); //dx就是当前车辆和目标点的x坐标之差
  const double dy = y - target_point.path_point().y(); //dy就是当前车辆和目标点的y坐标之差

  debug->mutable_current_target_point()->mutable_path_point()->set_x(target_point.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(target_point.path_point().y());

  ADEBUG << "x point: " << x << " y point: " << y;
  ADEBUG << "match point information : " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point().theta());
  const double sin_target_heading = std::sin(target_point.path_point().theta());

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  if (lat_based_lqr_controller_conf_.enable_navigation_mode_error_filter()) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }

  debug->set_lateral_error(lateral_error);

  debug->set_ref_heading(target_point.path_point().theta());

  //计算航向误差，车辆当前航向角theta-ref_heading角，然后再调用NormalizeAngle()函数将角度标准化，对于车辆航向角，通常在-pi-pi之间
  double heading_error = common::math::NormalizeAngle(theta - debug->ref_heading());
  if (lat_based_lqr_controller_conf_.enable_navigation_mode_error_filter()) {
    heading_error = heading_error_filter_.Update(heading_error);
  }
  debug->set_heading_error(heading_error);

  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  // 低速时根据预瞄点优化横向偏差和航向偏差
  double lookahead_station = 0.0;//ahead是针对非倒档
  double lookback_station = 0.0; //back是针对倒档
  if (std::fabs(linear_v) >= low_speed_bound_) { //如果速度的绝对值>=low_speed_bound_(低速边界) 
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {//若纵向速度绝对值小于低速边界-低速窗口
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {//若纵向速度绝对值小于低速边界又大于（低速边界-低速窗口）就插值计算预瞄距离
    lookahead_station = common::math::lerp(lookahead_station_low_speed_, 
                                           low_speed_bound_ - low_speed_window_,
                                           lookahead_station_high_speed_, 
                                           low_speed_bound_, 
                                           std::fabs(linear_v));
    lookback_station = common::math::lerp(lookback_station_low_speed_, 
                                          low_speed_bound_ - low_speed_window_,
                                          lookback_station_high_speed_, 
                                          low_speed_bound_, 
                                          std::fabs(linear_v));
  }

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = heading_error;
  } else {
    //目标点的相对时间 + 预瞄时间(预瞄距离/车辆纵向速度)作为总相对时间
	  //然后去trajectory_analyzer轨迹信息上根据总相对时间找出预瞄点
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(target_point.relative_time() +
                                                                               lookahead_station / (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));//在估计预瞄时间时纵向速度若小于0.1就按0.1
       //heading_error=车辆当前heading-参考点heading
    heading_error_feedback = common::math::NormalizeAngle(heading_error + target_point.path_point().theta() -
                                                          lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    //倒档的lateral_error_feedback=lateral_error-倒车预瞄距离*sin(heading_error)
    lateral_error_feedback = lateral_error - lookback_station * std::sin(heading_error);
  } else {
    //前进档的lateral_error_feedback=lateral_error+前进预瞄距离*sin(heading_error)
    lateral_error_feedback = lateral_error + lookahead_station * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);  //横向误差率=纵向速度v*sin(heading_error)
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);//横向误差加速度=纵向加速度*sin(heading_error)
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  auto centripetal_acceleration = linear_v * linear_v / wheelbase_ *
                                  std::tan(chassis->steering_percentage() / 100 * vehicle_param_.max_steer_angle() / steer_ratio_);
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_centripetal_acceleration(centripetal_acceleration);
  debug->set_lateral_jerk((debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_); //利用横向加速度差分得到横向加加速度jerk
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  //参考的航向角变化率=目标点纵向速度/目标点转弯半径，绕Z轴w=v/R，下面的kappa就是曲率
  debug->set_ref_heading_rate(target_point.path_point().kappa() *
                              target_point.v());
  //航向角误差率=车辆的航向角变化率-目标点航向角变化率，
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  //航向角变化的加速度就用差分法，这一时刻航向角变化率减去上一时刻之差然后再处以采样周期ts
  debug->set_heading_acceleration((debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration((debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() - debug->ref_heading_acceleration());

  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  //heading角的加加速度，同样差分法
  debug->set_heading_jerk((debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk((debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) / ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() - debug->ref_heading_jerk());

  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

  debug->set_curvature(target_point.path_point().kappa());
}

void LatController::UpdateDrivingOrientation() {
  auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation_ = common::math::NormalizeAngle(driving_orientation_ + M_PI);
      // Update Matrix_b for reverse mode
      matrix_bd_ = -matrix_b_ * ts_;
      ADEBUG << "Matrix_b changed due to gear direction";
    }
  }
}

}  // namespace control
}  // namespace apollo
