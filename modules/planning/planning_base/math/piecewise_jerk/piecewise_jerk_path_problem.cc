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

#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(const size_t num_of_knots, const double delta_s,
                                                   const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;                                             // l  l' l''
  const int num_of_nonzeros = num_of_variables + (n - 1);                         // 非零元素个数
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);  // P矩阵
  int value_index = 0;

  // ============================================ Px ============================================
  // (w_x + w_x_ref[i]) * x(i)^2
  // w_x_ref might be a uniform value for all x(i) or piecewise values for different x(i)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                               (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // (w_x + w_x_ref[n-1] + w_end_x) * x(n-1)^2  终点状态的权重项
  columns[n - 1].emplace_back(n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                                     (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // ============================================ Px' ============================================
  // w_dx * x(i)'^2
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(n + i, weight_dx_ / 
                                       (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // (w_dx + w_end_dx) * x(n-1)'^2
  columns[2 * n - 1].emplace_back(2 * n - 1, (weight_dx_ + weight_end_state_[1]) /
                                             (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  // ============================================ Px'' ============================================
  // (w_ddx + w_dddx / delta_s^2) * x(0)''^2
  auto delta_s_square = delta_s_ * delta_s_;
  columns[2 * n].emplace_back(2 * n, (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                     (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  // (w_ddx + 2 * w_dddx / delta_s^2) * x(i)''^2
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                                               (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  // (w_ddx + w_dddx / delta_s^2) * x(n-1)''^2
  columns[3 * n - 1].emplace_back(3 * n - 1,(weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
                                            (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)'' // ! 这里不应该乘以2  
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1, (-2.0 * weight_dddx_ / delta_s_square) /    
                                                   (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  CHECK_EQ(value_index, num_of_nonzeros);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);                      // 将即上一列的非零元素数量推入 P_indptr 中，表示当前列的起始位置。

    for (const auto& row_data_pair : columns[i]) {   // std::pair<c_int, c_float>
      P_data->push_back(row_data_pair.second * 2.0); // 每个元素前都乘以了2，这是为了和二次优化问题的一般形式中的1/2进行抵消的。
      P_indices->push_back(row_data_pair.first);     // 将当前非零元素的行索引存入 P_indices 中。
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  // l_ref
  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }

  // l_end_ref
  if (has_end_state_ref_) {
    q->at(n - 1) += -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) += -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) += -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

}  // namespace planning
}  // namespace apollo
