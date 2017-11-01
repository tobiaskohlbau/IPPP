//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#ifndef TYPES_H
#define TYPES_H

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ippp {

// global definitions
const double EPSILON = 0.00001;

// Eigen Vectors
template <unsigned int dim>
using Vector = Eigen::Matrix<double, dim, 1>;
using Vector2 = Eigen::Matrix<double, 2, 1>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector4 = Eigen::Matrix<double, 4, 1>;
using Vector5 = Eigen::Matrix<double, 5, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using VectorX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

// Eigen Matrices
template <unsigned int dim>
using Matrix = Eigen::Matrix<double, dim, dim>;
using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using MatrixX = Eigen::MatrixXd;
using Vector3i = Eigen::Matrix<int, 3, 1>;

using AABB = Eigen::AlignedBox<double, 3>;
using Transform = Eigen::Transform<double, 3, Eigen::AffineCompact>;
using Translation = Eigen::Translation<double, 3>;
using Rotation2D = Eigen::Rotation2D<double>;

} /* namespace ippp */

#endif    // TYPES_H
