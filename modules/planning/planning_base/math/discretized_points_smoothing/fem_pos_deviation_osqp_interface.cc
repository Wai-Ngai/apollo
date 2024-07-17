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
 * @file
 **/

#include "modules/planning/planning_base/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h"

#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool FemPosDeviationOsqpInterface::Solve() {
  // Sanity Check
  if (ref_points_.empty()) {
    AERROR << "reference points empty, solver early terminates";
    return false;
  }

  if (ref_points_.size() != bounds_around_refs_.size()) {
    AERROR << "ref_points and bounds size not equal, solver early terminates";
    return false;
  }

  if (ref_points_.size() < 3) {
    AERROR << "ref_points size smaller than 3, solver early terminates";
    return false;
  }

  if (ref_points_.size() > std::numeric_limits<int>::max()) {
    AERROR << "ref_points size too large, solver early terminates";
    return false;
  }

  // 二次规划 (1/2) * x' * P * x + q' * x

  // Calculate optimization states definitions
  num_of_points_ = static_cast<int>(ref_points_.size()); // 待优化变量包括n个坐标,因此共有2 * n个待优化变量。
  num_of_variables_ = num_of_points_ * 2;
  num_of_constraints_ = num_of_variables_;

  // Calculate kernel    
  std::vector<c_float> P_data;   // 二次型矩阵P
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate affine constraints   
  std::vector<c_float> A_data;   // 约束矩阵A
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;  // lb
  std::vector<c_float> upper_bounds;  // ub
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, 
                            &lower_bounds,
                            &upper_bounds);

  // Calculate offset  
  std::vector<c_float> q;
  CalculateOffset(&q);   // 一次矩阵q

  // Set primal warm start  OSQP还需要设定迭代初值，设定迭代初值为原始参考点坐标
  std::vector<c_float> primal_warm_start;
  SetPrimalWarmStart(&primal_warm_start);

  // 堆上分配 OSQPData 和 OSQPSettings 类型的内存空间，并返回对应的指针
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter_;
  settings->time_limit = time_limit_;                  // 求解器的时间限制
  settings->verbose = verbose_;                        // 是否输出详细的求解过程信息
  settings->scaled_termination = scaled_termination_;  // 是否启用缩放的终止条件
  settings->warm_start = warm_start_;                  // 利用上一次求解的结果作为当前求解的初始点，从而加速求解过程

  OSQPWorkspace* work = nullptr;

  //调用开源框架优化
  bool res = OptimizeWithOsqp(num_of_variables_, lower_bounds.size(), &P_data,
                              &P_indices, &P_indptr, &A_data, &A_indices,
                              &A_indptr, &lower_bounds, &upper_bounds, &q,
                              &primal_warm_start, data, &work, settings);
  if (res == false || work == nullptr || work->solution == nullptr) {
    AERROR << "Failed to find solution.";
    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);    // 释放在堆上开辟的内存
    c_free(data->P);
    c_free(data);
    c_free(settings);

    return false;
  }

  // Extract primal results
  x_.resize(num_of_points_);
  y_.resize(num_of_points_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    x_.at(i) = work->solution->x[index];
    y_.at(i) = work->solution->x[index + 1];
  }

  // Cleanup  释放在堆上开辟的内存
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void FemPosDeviationOsqpInterface::CalculateKernel(std::vector<c_float>* P_data, 
                                                   std::vector<c_int>* P_indices,
                                                   std::vector<c_int>* P_indptr) {
  CHECK_GT(num_of_variables_, 4); // glog 判断变量个数大于4

  // Three quadratic penalties are involved:
  // 1. Penalty x on distance between middle point and point by finite element estimate; 光滑代价
  // 2. Penalty y on path length; 均匀代价
  // 3. Penalty z on difference between points and reference points 与原始参考线相似代价

  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix

  // |X+Y+Z, -2X-Y,   X,       0,       0,       0    |
  // |0,     5X+2Y+Z, -4X-Y,   X,       0,       0    |
  // |0,     0,       6X+2Y+Z, -4X-Y,   X,       0    |
  // |0,     0,       0,       6X+2Y+Z, -4X-Y,   X    |
  // |0,     0,       0,       0,       5X+2Y+Z, -2X-Y|
  // |0,     0,       0,       0,       0,       X+Y+Z|

  // Only upper triangle needs to be filled 只考虑上三角是因为上下是对称的,只上三角结果是相同的
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_of_variables_);
  int col_num = 0;

  // 根据数值的规律，P矩阵有数值区域可以分为五个部分：第1、2列；第3、4列；倒数第1、2列；倒数第3、4列；中间所有列；
  // 第1、2列：X + Y + Z；之所以有两列，是因为坐标有x和y两个变量
  for (int col = 0; col < 2; ++col) {
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                   weight_path_length_ +
                                   weight_ref_deviation_); // x的第1行、y第2行
    ++col_num;
  }

  // 第3、4列：-2X-Y & 5X + 2Y + Z
  for (int col = 2; col < 4; ++col) {
    columns[col].emplace_back(col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_); // x的第1行、y第2行

    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                   2.0 * weight_path_length_ +
                                   weight_ref_deviation_); // x的第3行、y第4行
    ++col_num;
  }

  // 第5列~第num_of_points_-2列: X & -4X -Y & 6X + 2Y + Z
  int second_point_from_last_index = num_of_points_ - 2; 

  for (int point_index = 2; point_index < second_point_from_last_index; ++point_index) {
    int col_index = point_index * 2;

    for (int col = 0; col < 2; ++col) {
      col_index += col;
      columns[col_index].emplace_back(col_index - 4, weight_fem_pos_deviation_);

      columns[col_index].emplace_back(col_index - 2, -4.0 * weight_fem_pos_deviation_ - 
                                                     weight_path_length_);

      columns[col_index].emplace_back(col_index, 6.0 * weight_fem_pos_deviation_ +
                                                 2.0 * weight_path_length_ + 
                                                 weight_ref_deviation_);
      ++col_num;
    }
  }

  // 倒数第3、4列：-2X-Y & 5X + 2Y + Z
  int second_point_col_from_last_col = num_of_variables_ - 4;
  int last_point_col_from_last_col = num_of_variables_ - 2;

  for (int col = second_point_col_from_last_col; col < last_point_col_from_last_col; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);

    columns[col].emplace_back(col - 2, -4.0 * weight_fem_pos_deviation_ - 
                                       weight_path_length_);

    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                   2.0 * weight_path_length_ +
                                   weight_ref_deviation_);
    ++col_num;
  }

  // 倒数第1、2列：X + Y + Z
  for (int col = last_point_col_from_last_col; col < num_of_variables_; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);

    columns[col].emplace_back(col - 2, -2.0 * weight_fem_pos_deviation_ - 
                                       weight_path_length_);

    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                   weight_path_length_ +
                                   weight_ref_deviation_);
    ++col_num;
  }

  CHECK_EQ(col_num, num_of_variables_);

  //  CSC矩阵处理,按列存储稀疏矩阵
  int ind_p = 0;
  for (int i = 0; i < col_num; ++i) {
    P_indptr->push_back(ind_p);                  // 代表矩阵中每一列所存储数据在data中的开始和结束的索引
    for (const auto& row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem setup is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);  // 代表对应的data中的数据在其所在列中的所在行数
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void FemPosDeviationOsqpInterface::CalculateOffset(std::vector<c_float>* q) {
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.first);    // -2*W2*Xref
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.second);   // -2*W2*Yref
  }
}

void FemPosDeviationOsqpInterface::CalculateAffineConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
                                                             std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
                                                             std::vector<c_float>* upper_bounds) {
  // 构造A矩阵
  int ind_A = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    A_data->push_back(1.0);      // 全为1的列向量
    A_indices->push_back(i);
    A_indptr->push_back(ind_A);
    ++ind_A;
  }
  A_indptr->push_back(ind_A);

  // 构造Lb 和 Ub
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    upper_bounds->push_back(ref_point_xy.first + bounds_around_refs_[i]);    // Xref + buff
    upper_bounds->push_back(ref_point_xy.second + bounds_around_refs_[i]);   // Yref + buff
    lower_bounds->push_back(ref_point_xy.first - bounds_around_refs_[i]);    // Xref - buff
    lower_bounds->push_back(ref_point_xy.second - bounds_around_refs_[i]);   // Xref - buff
  }
}

void FemPosDeviationOsqpInterface::SetPrimalWarmStart(std::vector<c_float>* primal_warm_start) {
  CHECK_EQ(ref_points_.size(), static_cast<size_t>(num_of_points_));

  for (const auto& ref_point_xy : ref_points_) {
    primal_warm_start->push_back(ref_point_xy.first);   // Xref
    primal_warm_start->push_back(ref_point_xy.second);  // Yref
  }
}

bool FemPosDeviationOsqpInterface::OptimizeWithOsqp(const size_t kernel_dim, const size_t num_affine_constraint,
                                                    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
                                                    std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
                                                    std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
                                                    std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
                                                    std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
                                                    OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings) {
  CHECK_EQ(lower_bounds->size(), upper_bounds->size());

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data->size(), P_data->data(), // CSC是按列存储稀疏矩阵
                       P_indices->data(), P_indptr->data());
  data->q = q->data();
  data->A = csc_matrix(data->m, data->n, A_data->size(), A_data->data(),
                       A_indices->data(), A_indptr->data());
  data->l = lower_bounds->data();
  data->u = upper_bounds->data();

  *work = osqp_setup(data, settings);
  // osqp_setup(work, data, settings);

  osqp_warm_start_x(*work, primal_warm_start->data());

  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;

  if (status < 0) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
