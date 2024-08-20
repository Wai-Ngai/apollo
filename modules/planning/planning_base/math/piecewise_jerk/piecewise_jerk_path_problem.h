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

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_problem.h"

namespace apollo {
namespace planning {

/*
 * @brief:
 * FEM stands for finite element method.
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
 public:
 /**
  * @brief 构造函数
  * 
  * @param num_of_knots 离散点数量：横向边界采样点的数量
  * @param delta_s      离散点纵向间隔1.0m
  * @param x_init       横向的初始状态d0,d0',d0''
  */
  PiecewiseJerkPathProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkPathProblem() = default;

  void set_extra_constraints(const InterPolatedPointVec& extra_constraints) {
    extra_constraints_ = extra_constraints;
  }

  void set_vertex_constraints(const ADCVertexConstraints& vertexs) {
    vertex_constraints_ = vertexs;
  }

 protected:
  /**
   * @brief 构建二次规划问题的P矩阵，以CSC稀疏矩阵的格式储存到输入的3个指针数组里
   * 
   * @param P_data    P矩阵上面行列索引对应的元素值的数组
   * @param P_indices P矩阵行索引数组
   * @param P_indptr  P矩阵列索引数组
   */
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  /**
   * @brief 构建二次规划问题的q矩阵，构建结果储存到输入的指针q里
   * 
   * @param q 
   */
  void CalculateOffset(std::vector<c_float>* q) override;

 private:
  InterPolatedPointVec extra_constraints_;
  ADCVertexConstraints vertex_constraints_;  // 顶点约束
};

}  // namespace planning
}  // namespace apollo
