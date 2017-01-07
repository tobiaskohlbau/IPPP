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

#ifndef UTILGEO_H
#define UTILGEO_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <core/types.h>
#include <core/utility/UtilVec.hpp>

namespace rmpl {
namespace utilGeo {

constexpr float pi() {
    return std::atan(1) * 4;
}

constexpr float twoPi() {
    return std::atan(1) * 8;
}

constexpr float toRad() {
    return (std::atan(1) * 4) / 180;
}

constexpr float toDeg() {
    return 180 / (std::atan(1) * 4);
}

/*!
*  \brief      Create transformation matrix T from rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  rotation matrix
*  \param[in]  translatin matrix
*  \param[out] transformation matrix
*  \date       2016-08-25
*/
static Matrix4 createT(Matrix3 &R, Vector3 &t) {
    Matrix4 T = Eigen::Matrix4f::Identity(4, 4);
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

/*!
*  \brief      Decompose transformation matrix T in rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \param[out] rotation matrix
*  \param[out] translatin matrix
*  \date       2016-08-25
*/
static void decomposeT(Matrix4 &T, Matrix3 &R, Vector3 &t) {
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
}

/*!
*  \brief      Create 2D rotation matrix from deg
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rotation matrix
*  \date       2016-11-15
*/
static Matrix2 getRotMat2D(float deg) {
    Eigen::Rotation2D<float> rot2(deg * toRad());
    return rot2.toRotationMatrix();
}

/*!
*  \brief      Create 3D rotation matrix from deg
*  \author     Sascha Kaden
*  \param[in]  deg in x direction
*  \param[in]  deg in y direction
*  \param[in]  deg in z direction
*  \param[out] rotation matrix
*  \date       2016-11-15
*/
static Matrix3 getRotMat3D(float degX, float degY, float degZ) {
    Matrix3 R;
    R = Eigen::AngleAxisf(degX * toRad(), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(degY * toRad(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(degZ * toRad(), Eigen::Vector3f::UnitZ());
    return R;
}

/*!
*  \brief      Convert pose Vec to R and t in 2D
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] rotation matrix
*  \param[out] translation vector
*  \date       2016-11-15
*/
static void poseVecToRandT(const Vector3 &pose, Matrix2 &R, Vector2 &t) {
    R = getRotMat2D(pose[2]);
    t(0) = pose[0];
    t(1) = pose[1];
}

/*!
*  \brief      Convert pose Vec to R and t in 3D
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] rotation matrix
*  \param[out] translation vector
*  \date       2016-11-15
*/
static void poseVecToRandT(const Vector6 &pose, Matrix3 &R, Vector3 &t) {
    R = getRotMat3D(pose[3], pose[4], pose[5]);
    for (unsigned int i = 0; i < 3; ++i)
        t(i) = pose[i];
}

/*!
*  \brief      Convert pose Vec to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
static Matrix4 poseVecToMat(const Vector6 &pose) {
    Matrix3 R = getRotMat3D(pose[3], pose[4], pose[5]);
    Matrix4 T = Eigen::Matrix4f::Identity(4, 4);
    T.block<3, 3>(0, 0) = R;
    for (int i = 0; i < 3; ++i)
        T(i, 3) = pose[i];
    return T;
}

/*!
*  \brief      Convert transformation matrix into poseVec
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \param[out] pose Vector (angles)
*  \date       2016-07-07
*/
static Vector6 poseMatToVec(const Matrix4 &pose) {
    Vector3 vec(pose.block<3, 1>(0, 3));
    Vector3 euler(pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2));
    for (unsigned int i = 0; i < 3; ++i)
        euler(i, 0) *= toDeg();
    return utilVec::append<3, 3>(vec, euler);
}

/*!
*  \brief      Convert Vec of deg angles to Vec of rad
*  \author     Sascha Kaden
*  \param[in]  Vector of deg
*  \param[out] Vector of rad
*  \date       2016-07-07
*/
template <unsigned int dim>
Vector<dim> degToRad(Vector<dim> deg) {
    for (unsigned int i = 0; i < dim; ++i)
        deg[i] *= toRad();
    return deg;
}

/*!
*  \brief      Convert degree to radian
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rad
*  \date       2016-11-16
*/
static float degToRad(float deg) {
    return deg * toRad();
}

} /* namespace utilGeo */
} /* namespace rmpl */

#endif    // UTILGEO_H
