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

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {

MpcOsqp::MpcOsqp(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x, // 初始状态 x(0)
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
                 const int horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),    // 零向量
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
  state_dim_ = matrix_b.rows();
  control_dim_ = matrix_b.cols();
  ADEBUG << "state_dim" << state_dim_;
  ADEBUG << "control_dim_" << control_dim_;
  num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_; // 6*11 + 2*10 = 86 我们需要同时优化：86个优化变量
}

// 构建QP问题中的Hessian矩阵P
void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,   // 非零元素的值
                              std::vector<c_int> *P_indices,  // 每个非零元素的行索引
                              std::vector<c_int> *P_indptr) { // 以累加的方式存储每一列包含有非零数字的个数，从0开始
  // col1:(row,val),...; (行索引, 值)
  // col2:(row,val),....; 
  // ...
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;  // 是一个向量，每个元素代表P矩阵的一列
  columns.resize(num_param_);
  size_t value_index = 0;
  
  // 状态代价部分
  // state and terminal state
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      // (row, val)
      columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                               matrix_q_(j, j));
      ++value_index;
    }
  }

  // 控制代价部分
  // control
  const size_t state_total_dim = state_dim_ * (horizon_ + 1); // 控制部分起始索引
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      // (row, val)
      columns[i * control_dim_ + j + state_total_dim].emplace_back(state_total_dim + i * control_dim_ + j, 
                                                                   matrix_r_(j, j));
      ++value_index;
    }
  }
  CHECK_EQ(value_index, num_param_);  // 完整性检查:验证我们确实处理了所有变量

  // 转换为CSC格式
  int ind_p = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    // TODO(SHU) Check this
    P_indptr->emplace_back(ind_p);                   // 以累加的方式存储每一列包含有非零数字的个数，从0开始
    for (const auto &row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second);    // val 值
      P_indices->emplace_back(row_data_pair.first);  // row 行索引
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);  // 最后一个的位置
}

// reference is always zero
// 计算梯度向量q
void MpcOsqp::CalculateGradient() {
  // populate the gradient vector 
  gradient_ = Eigen::VectorXd::Zero(state_dim_ * (horizon_ + 1) + 
                                    control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_ + 1; i++) {
    gradient_.block(i * state_dim_, 0, state_dim_, 1) = -1.0 * matrix_q_ * matrix_x_ref_; // matrix_x_ref_ = 0 // 起始行↑   ↑起始列   ↑行数  ↑列数
  }
  ADEBUG << "Gradient_mat";
  ADEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
  static constexpr double kEpsilon = 1e-6;
  // block matrix  约束矩阵Ac
  Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_,  // 12(N+1) + 2N
                                                            state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);                               //  6(N+1) + 2N
  
  // 填充矩阵Ac
  Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1)); //  6(N+1) x 6(N+1)
  ADEBUG << "state_identity_mat" << state_identity_mat;

  matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                          state_dim_ * (horizon_ + 1)) = -1 * state_identity_mat; // 填充所有 -I
  ADEBUG << "matrix_constraint";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd control_identity_mat = Eigen::MatrixXd::Identity(control_dim_, control_dim_); // 没用

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, 
                            state_dim_, state_dim_) = matrix_a_;     // 填充所有 A   6 X 6
  }
  ADEBUG << "matrix_constraint with A";
  ADEBUG << matrix_constraint;

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_,
                            i * control_dim_ + (horizon_ + 1) * state_dim_,
                            state_dim_, control_dim_) = matrix_b_;   // 填充所有 B   6 X 2
  }
  ADEBUG << "matrix_constraint with B";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd all_identity_mat = Eigen::MatrixXd::Identity(num_param_, num_param_);

  matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, 
                          num_param_, num_param_) = all_identity_mat;  // 填充所有I   (6(N+1) + 2N) X (6(N+1) + 2N)
  ADEBUG << "matrix_constraint with I";
  ADEBUG << matrix_constraint;

  // 转成CSC格式
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_ + 1);
  int value_index = 0;
  // state and terminal state  遍历稠密矩阵，收集非零元素
  for (size_t i = 0; i < num_param_; ++i) {  // col 遍历每一列 6(N+1) + 2N
    for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1); ++j)  // row 遍历每一行 12(N+1) + 2N
      if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
        // (row, val)
        columns[i].emplace_back(j, matrix_constraint(j, i)); // 存储(行索引, 值)
        ++value_index;
      }
  }
  ADEBUG << "value_index";
  ADEBUG << value_index;
  int ind_A = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    A_indptr->emplace_back(ind_A);
    for (const auto &row_data_pair : columns[i]) {
      A_data->emplace_back(row_data_pair.second);    // value
      A_indices->emplace_back(row_data_pair.first);  // row
      ++ind_A;
    }
  }
  A_indptr->emplace_back(ind_A);  // 最后一列结束位置
}

// 计算约束边界向量 l u
void MpcOsqp::CalculateConstraintVectors() {
  // evaluate the lower and the upper inequality vectors  不等式约束向量
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  
  for (size_t i = 0; i < horizon_; i++) { // 控制约束设置
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_upper_;
  }
  ADEBUG << " matrix_u_lower_";

  for (size_t i = 0; i < horizon_ + 1; i++) { // 状态约束设置
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
  }
  ADEBUG << " matrix_x_lower_";

  // evaluate the lower and the upper equality vectors  等式约束向量
  Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_; // 等式约束只需要处理初始条件：x(0) = x_init  动力学约束 x(k+1) = A x(k) + B u(k) 会在约束矩阵 A 中处理
  upperEquality = lowerEquality;                                     // 上下界相等 → 等式约束
  lowerEquality = lowerEquality;
  ADEBUG << " matrix_initial_x_";

  // merge inequality and equality vectors 合并所有约束
  lowerBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;
  ADEBUG << " lowerBound_ ";

  upperBound_ = Eigen::MatrixXd::Zero(2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
  ADEBUG << " upperBound_";
}

OSQPSettings *MpcOsqp::Settings() {
  // default setting
  OSQPSettings *settings = reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  if (settings == nullptr) {
    return nullptr;
  } else {
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
  }
}

OSQPData *MpcOsqp::Data() {
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));     // OSQP数据结构初始化

  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;                // 优化变量的维度 = 状态变量数 + 控制变量数 86
  size_t num_affine_constraint = 2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_; // 约束的总数 = 等式约束 + 不等式约束 152

  if (data == nullptr) {
    return nullptr;
  } else {
    // 1. 设置优化
    data->n = kernel_dim;
    data->m = num_affine_constraint;

    // 2. 目标函数矩阵P的构建
    std::vector<c_float> P_data;   // 非零元素的值
    std::vector<c_int> P_indices;  // 每个非零元素的行索引
    std::vector<c_int> P_indptr;   // 以累加的方式存储每一列包含有非零数字的个数，从0开始
    ADEBUG << "before CalculateKernel";
    CalculateKernel(&P_data, &P_indices, &P_indptr);

    ADEBUG << "CalculateKernel done";

    // 创建CSC矩阵
    data->P = csc_matrix(kernel_dim,   // 矩阵行数
                         kernel_dim,   // 矩阵列数
                         P_data.size(), CopyData(P_data), // 最大非零元素数  // 非零元素值数组
                         CopyData(P_indices), CopyData(P_indptr));
    ADEBUG << "Get P matrix";

    // 3. 梯度向量q的设置
    data->q = gradient_.data();
    ADEBUG << "before CalculateEqualityConstraint";

    // 4. 约束矩阵A的构建
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);

    ADEBUG << "CalculateEqualityConstraint done";
    data->A = csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, // 行数
                         kernel_dim,                                                                          // 列数
                         A_data.size(), CopyData(A_data),
                         CopyData(A_indices), CopyData(A_indptr));

    ADEBUG << "Get A matrix";

    // 5. 约束边界向量的设置
    data->l = lowerBound_.data();
    data->u = upperBound_.data();
    return data;
  }
}

void MpcOsqp::FreeData(OSQPData *data) {
  c_free(data->A);
  c_free(data->P);
  c_free(data);
}

bool MpcOsqp::Solve(std::vector<double> *control_cmd) {
  ADEBUG << "Before Calc Gradient";
  CalculateGradient();           // 计算目标函数梯度（q向量）

  ADEBUG << "After Calc Gradient";

  CalculateConstraintVectors();  // 计算约束边界（l, u向量）
  ADEBUG << "MPC2Matrix";

  OSQPData *data = Data();       // 将内部数据结构转换为OSQP求解器所需格式  包含P, q, A, l, u
  ADEBUG << "OSQP data done";
  ADEBUG << "OSQP data n" << data->n;
  ADEBUG << "OSQP data m" << data->m;
  for (int i = 0; i < data->n; ++i) {
    ADEBUG << "OSQP data q" << i << ":" << (data->q)[i];
  }
  ADEBUG << "OSQP data l" << data->l;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data l" << i << ":" << (data->l)[i];
  }
  ADEBUG << "OSQP data u" << data->u;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data u" << i << ":" << (data->u)[i];
  }

  OSQPSettings *settings = Settings(); // 配置求解器设置
  ADEBUG << "OSQP setting done";

  OSQPWorkspace *osqp_workspace = nullptr;
  // osqp_setup(&osqp_workspace, data, settings);
  osqp_workspace = osqp_setup(data, settings);  // 创建求解器工作空间并求解
  ADEBUG << "OSQP workspace ready";
  osqp_solve(osqp_workspace);

  auto status = osqp_workspace->info->status_val;
  ADEBUG << "status:" << status;
  // check status
  if (status < 0 || (status != 1 && status != 2)) {   // 求解失败处理
    AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }

  size_t first_control = state_dim_ * (horizon_ + 1); // 提取控制命令
  for (size_t i = 0; i < control_dim_; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    ADEBUG << "control_cmd:" << i << ":" << control_cmd->at(i);
  }

  // Cleanup
  osqp_cleanup(osqp_workspace); // 清理资源
  FreeData(data);
  c_free(settings);
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
