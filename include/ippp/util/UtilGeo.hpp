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

#ifndef UTILGEO_HPP
#define UTILGEO_HPP

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ippp/types.h>


namespace ippp {
namespace util {

constexpr double pi() {
    return 3.141592653589793;
}

constexpr double twoPi() {
    return 3.141592653589793 * 2;
}

constexpr double halfPi() {
    return 3.141592653589793 / 2;
}

constexpr double toRad() {
    return 3.141592653589793 / 180;
}

constexpr double toDeg() {
    return 180 / 3.141592653589793;
}

Matrix2 getRotMat2D(double rad);
Matrix3 getRotMat3D(double radX, double radY, double radZ);

Transform createTransform(const Matrix3 &R, const Vector3 &t);
void decomposeT(const Matrix4 &T, Matrix3 &R, Vector3 &t);

Transform toTransform(const Vector6 &pose);
std::vector<Transform> toTransform(const std::vector<Vector6> poses);
Transform poseVecToTransformFromDeg(const Vector6 &pose);
Vector6 toPoseVec(const Transform &T);

Vector3 computeNormal(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3);
AABB transformAABB(const AABB &aabb, const Transform &T);
AABB translateAABB(const AABB &a, const Transform &T);

MatrixX transformToTaskFrameJ(const MatrixX &jacobian, const Transform taskFrame);

void removeDuplicates(std::vector<Vector3> &vectors);

double toDeg(double rad);
double toRad(double deg);

/*!
*  \brief      Convert Vec of deg angles to Vec of rad
*  \author     Sascha Kaden
*  \param[in]  Vector of deg
*  \param[out] Vector of rad
*  \date       2016-07-07
*/
template <unsigned int dim>
Vector<dim> toRad(Vector<dim> deg) {
    for (unsigned int i = 0; i < dim; ++i)
        deg[i] *= toRad();
    return deg;
}

/*!
*  \brief      Convert Vec of rad angles to Vec of deg
*  \author     Sascha Kaden
*  \param[in]  Vector of rad
*  \param[out] Vector of deg
*  \date       2016-07-07
*/
template <unsigned int dim>
Vector<dim> toDeg(Vector<dim> rad) {
    for (unsigned int i = 0; i < dim; ++i)
        rad[i] *= toDeg();
    return rad;
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILGEO_HPP
