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

#ifndef UTILGEO_HPP
#define UTILGEO_HPP

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <core/types.h>
#include <core/utility/UtilVec.hpp>

namespace rmpl {
namespace util {

constexpr float pi() {
    return M_PI;
}

constexpr float twoPi() {
    return M_PI * 2;
}

constexpr float halfPi() {
    return M_PI / 2;
}

constexpr float toRad() {
    return M_PI / 180;
}

constexpr float toDeg() {
    return 180 / M_PI;
}

/*!
*  \brief      Create transformation matrix T from rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  rotation matrix
*  \param[in]  translatin matrix
*  \param[out] transformation matrix
*  \date       2016-08-25
*/
static Matrix4 createT(const Matrix3 &R, const Vector3 &t) {
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
*  \param[out] translation matrix
*  \date       2016-08-25
*/
static void decomposeT(const Matrix4 &T, Matrix3 &R, Vector3 &t) {
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
}

/*!
*  \brief      Decompose transformation matrix T in rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \param[out] rotation matrix
*  \param[out] translation matrix
*  \date       2016-08-25
*/
static void decomposeT(const Matrix3 &T, Matrix2 &R, Vector2 &t) {
    R = T.block<2, 2>(0, 0);
    t = T.block<2, 1>(0, 2);
}

/*!
*  \brief      Create 2D rotation matrix from deg
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rotation matrix
*  \date       2016-11-15
*/
static Matrix2 getRotMat2D(const float rad) {
    Eigen::Rotation2D<float> R(rad);
    return R.toRotationMatrix();
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
static Matrix3 getRotMat3D(const float radX, const float radY, const float radZ) {
    Matrix3 R;
    R = Eigen::AngleAxisf(radX, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(radY, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(radZ, Eigen::Vector3f::UnitZ());
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
    for (unsigned int i = 0; i < 3; ++i) {
        t(i) = pose[i];
    }
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
    for (int i = 0; i < 3; ++i) {
        T(i, 3) = pose[i];
    }
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
    return util::append<3, 3>(vec, euler);
}

/*!
*  \brief      Compute the normal from the plane of three points.
*  \author     Sascha Kaden
*  \param[in]  point one
*  \param[in]  point two
*  \param[in]  point three
*  \param[out] normal Vector
*  \date       2017-04-07
*/
static Vector3 computeNormal(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3) {
    Vector3 v = p2 - p1;
    Vector3 w = p3 - p1;
    float nx = (v[1] * w[2]) - (v[2] * w[1]);
    float ny = (v[2] * w[0]) - (v[0] * w[2]);
    float nz = (v[0] * w[1]) - (v[1] * w[0]);
    Vector3 normal(nx, ny, nz);
    return normal.normalized();
}

/*!
*  \brief      Remove duplicate vectors from the passed reference list.
*  \author     Sascha Kaden
*  \param[in]  list of vectors
*  \date       2017-04-07
*/
static Vector3 removeDuplicates(std::vector<Vector3> &vectors) {
    // sort vector list
    struct {
        bool operator()(Vector3 a, Vector3 b) {
            return a.x() < b.x();
        }
    } customCompare;
    std::sort(vectors.begin(), vectors.end(), customCompare);

    // remove duplicates
    for (auto vec = vectors.begin(); vec != vectors.end(); ++vec) {
        int i = 1;
        while (vec + i != vectors.end() && vec->x() - (vec + i)->x() < 0.01) {
            if ((*vec - *(vec + i)).squaredNorm() < 0.0001) {
                vectors.erase(vec + i);
            } else {
                ++i;
            }
        }
    }
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
    for (unsigned int i = 0; i < dim; ++i) {
        deg[i] *= toRad();
    }
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
Vector<dim> radToDeg(Vector<dim> rad) {
    for (unsigned int i = 0; i < dim; ++i) {
        rad[i] *= toDeg();
    }
    return rad;
}

/*!
*  \brief      Convert degree to radian
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rad
*  \date       2016-11-16
*/
static float degToRad(const float deg) {
    return deg * toRad();
}

} /* namespace util */
} /* namespace rmpl */

#endif    // UTILGEO_HPP
