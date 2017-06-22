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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ippp/core/types.h>
#include <ippp/core/util/UtilVec.hpp>

namespace ippp {
namespace util {

constexpr double pi() {
    return M_PI;
}

constexpr double twoPi() {
    return M_PI * 2;
}

constexpr double halfPi() {
    return M_PI / 2;
}

constexpr double toRad() {
    return M_PI / 180;
}

constexpr double toDeg() {
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
    Matrix4 T = Matrix4::Identity(4, 4);
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
static Matrix2 getRotMat2D(const double rad) {
    Eigen::Rotation2D<double> R(rad);
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
static Matrix3 getRotMat3D(const double radX, const double radY, const double radZ) {
    Matrix3 R;
    R = Eigen::AngleAxisd(radX, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(radY, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(radZ, Eigen::Vector3d::UnitZ());
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
*  \brief      Convert pose Vec to R and t in 2D
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] rotation matrix and translation vector
*  \date       2017-06-20
*/
static std::pair<Matrix3, Vector3> poseVecToRandT(const Vector3 &pose) {
    Vector3 t(pose[0], pose[1], 0);
    Matrix3 R = Matrix3::Identity(3,3);
    R.block<2, 2>(0,0) = getRotMat2D(pose[2]);
    return std::make_pair(R, t);
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
*  \brief      Convert pose Vec to R and t in 2D
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] rotation matrix and translation vector
*  \date       2017-06-20
*/
static std::pair<Matrix3, Vector3> poseVecToRandT(const Vector6 &pose) {
    Vector3 t(pose[0], pose[1], pose[2]);
    Matrix3 R = getRotMat3D(pose[3], pose[4], pose[5]);
    return std::make_pair(R, t);
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
    Matrix4 T = Matrix4::Identity(4, 4);
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
    double nx = (v[1] * w[2]) - (v[2] * w[1]);
    double ny = (v[2] * w[0]) - (v[0] * w[2]);
    double nz = (v[0] * w[1]) - (v[1] * w[0]);
    Vector3 normal(nx, ny, nz);
    return normal.normalized();
}

/*!
*  \brief      Transforms a AABB with the passed transformations.
*  \details    The new AABB has a larger size as the original and the AABB is no more tight!
*  \author     Sascha Kaden
*  \param[in]  original aabb
*  \param[in]  pair with rotation and transformation
*  \param[out] transformed aabb
*  \date       2017-06-21
*/
static AABB transformAABB(const AABB &a, const std::pair<Matrix3,Vector3> &trafo)
{
    // trafo is pair with rotation and translation
    Vector3 center(trafo.second);
    Vector3 radius = Vector3::Zero(3,1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            center[i] += trafo.second(i,j) * a.center()[j];
            radius[i] += std::abs(trafo.second(i,j)) * a.diagonal()[j] / 2;
        }
    }
    // return AABB by new construction min and max point
    return AABB(center - radius, center + radius);
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
static double degToRad(const double deg) {
    return deg * toRad();
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILGEO_HPP
