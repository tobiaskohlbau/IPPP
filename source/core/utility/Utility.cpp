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

#include <core/utility/Utility.h>

#include <Eigen/Geometry>

namespace rmpl {
namespace utility {

/*!
*  \brief      Create transformation matrix T from rotation R and translation t
*  \author     Sascha Kaden
*  \param[in]  rotation matrix
*  \param[in]  translatin matrix
*  \param[out] transformation matrix
*  \date       2016-08-25
*/
Eigen::Matrix4f createT(Eigen::Matrix3f &R, Eigen::Vector3f &t) {
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
void decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
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
Eigen::Matrix2f getRotMat2D(float deg) {
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
Eigen::Matrix3f getRotMat3D(float degX, float degY, float degZ) {
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(degX * toRad(), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(degY * toRad(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(degZ * toRad(), Eigen::Vector3f::UnitZ());
    return R;
}

/*!
*  \brief      Convert pose Vec to R and t in 2D
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \param[out] rotation matrix
*  \param[out] translation vector
*  \date       2016-11-15
*/
void poseVecToRandT(const Eigen::Vector3f &pose, Eigen::Matrix2f &R, Eigen::Vector2f &t) {
    assert(pose.rows() == 3);
    R = getRotMat2D(pose[2]);
    t(0) = pose[0];
    t(1) = pose[1];
}

/*!
*  \brief      Convert pose Vec to R and t in 3D
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \param[out] rotation matrix
*  \param[out] translation vector
*  \date       2016-11-15
*/
void poseVecToRandT(const Eigen::Matrix<float, 6, 1> &pose, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
    assert(pose.rows() == 6);
    R = getRotMat3D(pose[3], pose[4], pose[5]);
    t(0) = pose[0];
    t(1) = pose[1];
    t(2) = pose[2];
}

/*!
*  \brief      Convert pose Vec to transformation matrix
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
Eigen::Matrix4f poseVecToMat(const Eigen::Matrix<float, 6, 1> &pose) {
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
*  \param[out] pose Vec (angles)
*  \date       2016-07-07
*/
Eigen::Matrix<float, 6, 1> poseMatToVec(const Eigen::Matrix4f &pose) {
    Eigen::Vector3f vec(pose(0, 3), pose(1, 3), pose(2, 3));
    Eigen::Vector3f euler = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    euler(0, 0) *= toDeg();
    euler(1, 0) *= toDeg();
    euler(2, 0) *= toDeg();
    return append(vec, euler);
}

/*!
*  \brief      Convert Vec of deg angles to Vec of rad
*  \author     Sascha Kaden
*  \param[in]  Vec of deg
*  \param[out] Vec of rad
*  \date       2016-07-07
*/
Eigen::VectorXf degToRad(Eigen::VectorXf deg) {
    Eigen::VectorXf rad(deg.rows());
    for (unsigned int i = 0; i < deg.rows(); ++i)
        rad[i] = deg[i] * toRad();
    return rad;
}

/*!
*  \brief      Convert degree to radian
*  \author     Sascha Kaden
*  \param[in]  deg
*  \param[out] rad
*  \date       2016-11-16
*/
float degToRad(float deg) {
    return deg * toRad();
}

/*!
*  \brief      Removes Node by reference object
*  \author     Sascha Kaden
*  \param[in]  list
*  \param[in]  pointer of Node
*  \date       2016-12-19
*/
void eraseFromList(std::vector<std::shared_ptr<Node>> &list, const std::shared_ptr<Node> &node) {
    list.erase(std::remove(list.begin(), list.end(), node), list.end());
}

/*!
*  \brief      Returns the Node with the lowest cost and removes it from the list
*  \author     Sascha Kaden
*  \param[in]  list
*  \param[out] Node with lowest cost
*  \date       2016-12-19
*/
std::shared_ptr<Node> removeMinFromList(std::vector<std::shared_ptr<Node>> &list) {
    float min = std::numeric_limits<float>::max();
    std::shared_ptr<Node> minNode = nullptr;
    for (int i = 0; i < list.size(); ++i) {
        if (list[i]->getCost() < min) {
            minNode = list[i];
            min = list[i]->getCost();
        }
    }
    if (minNode != nullptr)
        eraseFromList(list, minNode);

    return minNode;
}

/*!
*  \brief      Return true, if the list contains the passed Node
*  \author     Sascha Kaden
*  \param[in]  list
*  \param[in]  Node
*  \param[out] true, if list contains passed Node
*  \date       2016-12-19
*/
bool contains(std::vector<std::shared_ptr<Node>> &list, std::shared_ptr<Node> &node) {
    if (std::find(list.begin(), list.end(), node) != list.end())
        return true;
    else
        return false;
}

} /* namespace utility */

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

} /* namespace rmpl */