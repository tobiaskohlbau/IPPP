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

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <memory>

#include <Eigen/Core>
#include <PQP.h>

#include <core/dataObj/Node.h>

namespace rmpl {
namespace utility {

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

Eigen::Matrix4f createT(Eigen::Matrix3f &R, Eigen::Vector3f &t);
void decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t);

Eigen::Matrix2f getRotMat2D(float deg);
Eigen::Matrix3f getRotMat3D(float degX, float degY, float degZ);

void poseVecToRandT(const Eigen::Matrix<float, 6, 1> &pose, Eigen::Matrix3f &R, Eigen::Vector3f &t);
void poseVecToRandT(const Eigen::Vector3f &pose, Eigen::Matrix2f &R, Eigen::Vector2f &t);
Eigen::Matrix4f poseVecToMat(const Eigen::Matrix<float, 6, 1> &pose);
Eigen::Matrix<float, 6, 1> poseMatToVec(const Eigen::Matrix4f &pose);

Eigen::VectorXf degToRad(const Eigen::VectorXf deg);
float degToRad(float deg);

void eraseFromList(std::vector<std::shared_ptr<Node>> &list, const std::shared_ptr<Node> &node);
std::shared_ptr<Node> removeMinFromList(std::vector<std::shared_ptr<Node>> &list);
bool contains(std::vector<std::shared_ptr<Node>> &list, std::shared_ptr<Node> &node);

} /* namespace utility */

bool empty(Eigen::VectorXf vec);
Eigen::VectorXf append(Eigen::VectorXf source, Eigen::VectorXf add);
Eigen::Vector3f append(Eigen::Vector2f source, float add);
Eigen::Vector4f append(Eigen::Vector3f source, float add);
Eigen::VectorXf append(Eigen::VectorXf source, float add);

Eigen::VectorXf Vecf(unsigned int dim);
Eigen::Matrix<float, 5, 1> Vecf(float x, float y, float z, float rx, float ry);
Eigen::Matrix<float, 6, 1> Vecf(float x, float y, float z, float rx, float ry, float rz);
Eigen::VectorXf Vecf(unsigned int dim, float data[]);

} /* namespace rmpl */

#endif    // UTILITIES_H_
