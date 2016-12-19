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
#include <core/dataObj/Vec.hpp>

namespace rmpl {
namespace utility {

constexpr float pi() {
    return std::atan(1) * 4;
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

void poseVecToRandT(const Vec<float> &pose, Eigen::Matrix3f &R, Eigen::Vector3f &t);
void poseVecToRandT(const Vec<float> &pose, Eigen::Matrix2f &R, Eigen::Vector2f &t);
Eigen::Matrix4f poseVecToMat(const Vec<float> &pose);
Vec<float> poseMatToVec(const Eigen::Matrix4f &pose);

Vec<float> degToRad(const Vec<float> deg);
float degToRad(float deg);

Eigen::VectorXf VecToEigen(const Vec<float> &vec);
Eigen::VectorXf VecToEigen(const Vec<PQP_REAL> &vec);
Vec<float> EigenToVec(const Eigen::VectorXf &eigenVec);

void eraseFromList(std::vector<std::shared_ptr<Node>> &list, const std::shared_ptr<Node> &node);
std::shared_ptr<Node> removeMinFromList(std::vector<std::shared_ptr<Node>> &list);
bool contains(std::vector<std::shared_ptr<Node>> &list, std::shared_ptr<Node> &node);

} /* namespace utility */
} /* namespace rmpl */

#endif    // UTILITIES_H_
