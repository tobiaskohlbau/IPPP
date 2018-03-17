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

#include <ippp/util/UtilGeo.hpp>

#include <ippp/util/UtilVec.hpp>

namespace ippp {
namespace util {

/*!
*  \brief      Create 2D rotation matrix from rad
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rotation matrix
*  \date       2016-11-15
*/
Matrix2 getRotMat2D(double rad) {
    Eigen::Rotation2D<double> R(rad);
    return R.toRotationMatrix();
}

/*!
*  \brief      Create 3D rotation matrix from rad
*  \author     Sascha Kaden
*  \param[in]  deg in x direction
*  \param[in]  deg in y direction
*  \param[in]  deg in z direction
*  \param[out] rotation matrix
*  \date       2016-11-15
*/
Matrix3 getRotMat3D(double radX, double radY, double radZ) {
    Matrix3 R;
    R = Eigen::AngleAxisd(radX, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(radY, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(radZ, Eigen::Vector3d::UnitZ());
    return R;
}

/*!
*  \brief      Create transformation matrix T from rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  rotation matrix
*  \param[in]  translatin matrix
*  \param[out] transformation matrix
*  \date       2016-08-25
*/
Transform createTransform(const Matrix3 &R, const Vector3 &t) {
    Transform T;
    T = Translation(t) * R;
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
void decomposeT(const Matrix4 &T, Matrix3 &R, Vector3 &t) {
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
}

/*!
*  \brief      Convert pose config to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vector
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
Transform toTransform(const Vector6 &pose) {
    Transform T;
    T = Translation(Vector3(pose[0], pose[1], pose[2])) * Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
    return T;
}

/*!
*  \brief      Convert pose configs to transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  pose Vectors
*  \param[out] transformation matrizes
*  \date       2016-07-07
*/
std::vector<Transform> toTransform(const std::vector<Vector6> poses) {
    std::vector<Transform> transforms;
    transforms.reserve(poses.size());
    for (const auto &pose : poses)
        transforms.push_back(toTransform(pose));

    return transforms;
}

/*!
*  \brief      Convert pose config to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vector witr degree angles
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
Transform poseVecToTransformFromDeg(const Vector6 &pose) {
    auto poseVec = pose;
    for (size_t i = 3; i < 6; ++i)
        poseVec[i] *= toRad();
    return toTransform(poseVec);
}

/*!
*  \brief      Convert transformation matrix into poseVec
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \param[out] pose Vector (angles)
*  \date       2016-07-07
*/
Vector6 toPoseVec(const Transform &T) {
    Vector3 vec(T.translation());
    Vector3 euler(T.rotation().eulerAngles(0,1,2));
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
Vector3 computeNormal(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3) {
    Vector3 v = p2 - p1;
    Vector3 w = p3 - p1;
    double nx = (v[1] * w[2]) - (v[2] * w[1]);
    double ny = (v[2] * w[0]) - (v[0] * w[2]);
    double nz = (v[0] * w[1]) - (v[1] * w[0]);
    Vector3 normal(nx, ny, nz);
    return normal.normalized();
}

/*!
*  \brief      Transforms an AABB with the passed transformations and return the new AABB.
*  \details    The new AABB has a larger size as the original and the AABB is no more tight!
*  \author     Sascha Kaden
*  \param[in]  original AABB
*  \param[in]  Transform
*  \param[out] transformed AABB
*  \date       2017-06-21
*/
AABB transformAABB(const AABB &aabb, const Transform &T) {
    Vector3 min = aabb.min();
    Vector3 max = aabb.max();
    Vector4 min4 = util::append<3>(min, 1);
    Vector4 max4 = util::append<3>(max, 1);
    min4 = T * min4;
    max4 = T * max4;
    return AABB(Vector3(min4[0], min4[1], min4[2]), Vector3(max4[0], max4[1], max4[2]));

    //    Vector3 center(T.translation());
    //    Vector3 radius = Vector3::Zero(3, 1);
    //    for (size_t i = 0; i < 3; i++) {
    //        for (size_t j = 0; j < 3; j++) {
    //            center[i] += T(i, j) * aabb.center()[j];
    //            radius[i] += std::abs(T(i, j)) * aabb.diagonal()[j] / 2;
    //        }
    //    }
    //    return AABB(center - radius, center + radius);
}

/*!
*  \brief      Translate an AABB with the passed transformations.
*  \details    The new AABB has a larger size as the original and the AABB is no more tight!
*  \author     Sascha Kaden
*  \param[in]  original aabb
*  \param[in]  pair with rotation and transformation
*  \param[out] transformed aabb
*  \date       2017-06-21
*/
AABB translateAABB(const AABB &a, const Transform &T) {
    AABB result(a);
    result.translate(T.translation());
    return result;
}

MatrixX transformToTaskFrameJ(const MatrixX &jacobian, const Transform taskFrame) {
    Matrix6 toTaskFrame = Matrix6::Zero();
    Matrix3 R = taskFrame.rotation();
    toTaskFrame.block<3, 3>(0, 0) = R.inverse();
    toTaskFrame.block<3, 3>(3, 3) = R.inverse();

    Matrix6 E = Matrix6::Identity();
    //double psi = std::atan2(R(2, 1), R(2, 2));
    double theta = std::asin(R(2, 0));
    double phi = std::atan2(R(1, 0), R(0, 0));
    E(3, 3) = std::cos(phi) / std::cos(theta);
    E(3, 4) = std::sin(phi) / std::cos(theta);
    E(4, 3) = -std::sin(phi);
    E(4, 4) = std::cos(phi);
    E(5, 3) = (std::cos(phi) * std::sin(theta)) / std::cos(theta);
    E(5, 4) = (std::sin(phi) * std::sin(theta)) / std::cos(theta);

    MatrixX J = toTaskFrame * jacobian;
    return E * J;
}


/*!
*  \brief      Remove duplicate vectors from the passed reference list.
*  \author     Sascha Kaden
*  \param[in]  list of vectors
*  \date       2017-04-07
*/
void removeDuplicates(std::vector<Vector3> &vectors) {
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
            if ((*vec - *(vec + i)).squaredNorm() < 0.0001)
                vectors.erase(vec + i);
            else
                ++i;
        }
    }
}

/*!
*  \brief      Convert radian to degree value.
*  \author     Sascha Kaden
*  \param[in]  radian
*  \date       2017-04-07
*/
double toDeg(double rad) {
    return rad * toDeg();
}

/*!
*  \brief      Convert degree to radian value.
*  \author     Sascha Kaden
*  \param[in]  degree
*  \date       2017-04-07
*/
double toRad(double deg) {
    return deg * toRad();
}

} /* namespace util */
} /* namespace ippp */
