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

#include <core/Utilities.h>

#include <Eigen/Geometry>

using namespace rmpl;

/*!
*  \brief      Create transformation matrix T from rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  rotation matrix
*  \param[in]  translatin matrix
*  \param[out] transformation matrix
*  \date       2016-08-25
*/
Eigen::Matrix4f Utilities::createT(Eigen::Matrix3f &R, Eigen::Vector3f &t) {
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
void Utilities::decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
}

/*!
*  \brief      Convert pose Vec to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
Eigen::Matrix4f Utilities::poseVecToMat(const Vec<float> &pose) {
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(pose[3] / 180 * pi(), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pose[4] / 180 * pi(), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(pose[5] / 180 * pi(), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f mat = Eigen::Matrix4f::Zero(4, 4);
    mat.block<3, 3>(0, 0) = R;
    for (int i = 0; i < 3; ++i)
        mat(i, 3) = pose[i];
    mat(3, 3) = 1;
    return mat;
}

/*!
*  \brief      Convert transformation matrix into poseVec
*  \author     Sascha Kaden
*  \param[in]  transformation matrix
*  \param[out] pose Vec (angles)
*  \date       2016-07-07
*/
Vec<float> Utilities::poseMatToVec(const Eigen::Matrix4f &pose) {
    Vec<float> vec(pose(0, 3), pose(1, 3), pose(2, 3));
    Eigen::Vector3f euler = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    euler(0, 0) *= 180 / pi();
    euler(1, 0) *= 180 / pi();
    euler(2, 0) *= 180 / pi();
    vec.append(EigenToVec(euler));
    return vec;
}

/*!
*  \brief      Convert deg angles to rad
*  \author     Sascha Kaden
*  \param[in]  Vec of deg angles
*  \param[out] Vec of rad angles
*  \date       2016-07-07
*/
Vec<float> Utilities::degToRad(const Vec<float> deg) {
    Vec<float> rad(deg.getDim());
    for (unsigned int i = 0; i < deg.getDim(); ++i)
        rad[i] = deg[i] * pi() / 180;
    return rad;
}

float Utilities::degToRad(float deg) {
    return deg * pi() / 180;
}
/*!
*  \brief      Convert rmpl Vec to Eigen Array
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] Eigen Array
*  \date       2016-07-07
*/
Eigen::VectorXf Utilities::VecToEigen(const Vec<float> &vec) {
    Eigen::ArrayXf eigenVec(vec.getDim());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        eigenVec(i, 0) = vec[i];
    return eigenVec;
}

/*!
*  \brief      Convert rmpl Vec to Eigen Array
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] Eigen Array
*  \date       2016-07-07
*/
Eigen::VectorXf Utilities::VecToEigen(const Vec<PQP_REAL> &vec) {
    Eigen::ArrayXf eigenVec(vec.getDim());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        eigenVec(i, 0) = vec[i];
    return eigenVec;
}

/*!
*  \brief      Convert Eigen Array to rmpl Vec
*  \author     Sascha Kaden
*  \param[in]  Eigen Array
*  \param[out] Vec
*  \date       2016-07-07
*/
Vec<float> Utilities::EigenToVec(const Eigen::VectorXf &eigenVec) {
    Vec<float> vec((unsigned int)eigenVec.rows());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        vec[i] = eigenVec(i, 0);
    return vec;
}