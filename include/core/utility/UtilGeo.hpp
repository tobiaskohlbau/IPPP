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
static Eigen::Matrix4f createT(Eigen::Matrix3f &R, Vector3 &t) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4, 4);
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
static void decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Vector3 &t) {
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
static Eigen::Matrix2f getRotMat2D(float deg) {
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
static Eigen::Matrix3f getRotMat3D(float degX, float degY, float degZ) {
    Eigen::Matrix3f R;
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
static void poseVecToRandT(const Vector3 &pose, Eigen::Matrix2f &R, Vector2 &t) {
    assert(pose.rows() == 3);
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
static void poseVecToRandT(const Vector6 &pose, Eigen::Matrix3f &R, Vector3 &t) {
    assert(pose.rows() == 6);
    R = getRotMat3D(pose[3], pose[4], pose[5]);
    t(0) = pose[0];
    t(1) = pose[1];
    t(2) = pose[2];
}

/*!
*  \brief      Convert pose Vec to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
static Eigen::Matrix4f poseVecToMat(const Vector6 &pose) {
    Eigen::Matrix3f R = getRotMat3D(pose[3], pose[4], pose[5]);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4, 4);
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
static Vector6 poseMatToVec(const Eigen::Matrix4f &pose) {
    Vector3 vec(pose(0, 3), pose(1, 3), pose(2, 3));
    Vector3 euler = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    euler(0, 0) *= toDeg();
    euler(1, 0) *= toDeg();
    euler(2, 0) *= toDeg();
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
