//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include <Eigen/Dense>

namespace rmpl {

template <unsigned int dim>
using Vector = Eigen::Matrix<float, dim, 1>;
using Vector2 = Eigen::Matrix<float, 2, 1>;
using Vector3 = Eigen::Matrix<float, 3, 1>;
using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector5 = Eigen::Matrix<float, 5, 1>;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using VectorX = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Vector3i = Eigen::Matrix<int, 3, 1>;

using Matrix2 = Eigen::Matrix2f;
using Matrix3 = Eigen::Matrix3f;
using Matrix4 = Eigen::Matrix4f;
using MatrixX = Eigen::MatrixXf;


} /* namespace rmpl */


#endif //TYPES_H
