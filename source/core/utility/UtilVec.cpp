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

#include <core/utility/UtilVec.h>

namespace rmpl {
namespace utilVec {

bool empty(Eigen::VectorXf vec) {
    if (vec.rows() == 0 || vec.rows() == -1 || vec.rows() > 100)
        return true;
    else
        return false;
}

Eigen::VectorXf append(Eigen::VectorXf source, Eigen::VectorXf add) {
    Eigen::VectorXf vec;
    vec.resize(source.rows() + add.rows(), 1);
    for (int i = 0; i < source.rows(); ++i)
        vec[i] = source[i];
    for (int i = 0; i < add.rows(); ++i)
        vec[source.rows() + i] = add[i];
    return vec;
}

Eigen::Vector3f append(Eigen::Vector2f source, float add) {
    Eigen::Vector3f vec;
    for (int i = 0; i < source.rows(); ++i)
        vec[i] = source[i];
    vec[source.rows()] = add;
    return vec;
}

Eigen::Vector4f append(Eigen::Vector3f source, float add) {
    Eigen::Vector4f vec;
    for (int i = 0; i < source.rows(); ++i)
        vec[i] = source[i];
    vec[source.rows()] = add;
    return vec;
}

Eigen::VectorXf append(Eigen::VectorXf source, float add) {
    Eigen::VectorXf vec;
    vec.resize(source.rows() + 1, 1);
    for (int i = 0; i < source.rows(); ++i)
        vec[i] = source[i];

    vec[source.rows()] = add;
    return vec;
}

Eigen::VectorXf Vecf(unsigned int dim) {
    Eigen::VectorXf vec;
    vec.resize(dim, 1);
    return vec;
}

Eigen::Matrix<float, 5, 1> Vecf(float x, float y, float z, float rx, float ry) {
    Eigen::Matrix<float, 5, 1> vec;
    vec << x, y, z, rx, ry;
    return vec;
}

Eigen::Matrix<float, 6, 1> Vecf(float x, float y, float z, float rx, float ry, float rz) {
    Eigen::Matrix<float, 6, 1> vec;
    vec << x, y, z, rx, ry, rz;
    return vec;
}

Eigen::VectorXf Vecf(unsigned int dim, float data[]) {
    Eigen::VectorXf vec;
    vec.resize(dim, 1);
    for (unsigned int i = 0; i < dim; ++i)
        vec[i] = data[i];
    return vec;
}

} /* namespace utilVec */
} /* namespace rmpl */

