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

#include <Eigen/Core>
#include <PQP.h>

#include <core/dataObj/Vec.hpp>

namespace rmpl {

/*!
* \brief   Offers basical functions for matrix composition and conversation
* \author  Sascha Kaden
* \date    2016-07-07
*/
class Utilities {
  public:
    static constexpr float pi() {
        return std::atan(1) * 4;
    }

    static constexpr float toRad() {
        return (std::atan(1) * 4) / 180;
    }

    static constexpr float toDeg() {
        return 180 / (std::atan(1) * 4);
    }

    static Eigen::Matrix4f createT(Eigen::Matrix3f &R, Eigen::Vector3f &t);
    static void decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t);

    static Eigen::Matrix2f getRotMat2D(float deg);
    static Eigen::Matrix3f getRotMat3D(float degX, float degY, float degZ);

    static void poseVecToRandT(const Vec<float> &pose, Eigen::Matrix3f &R, Eigen::Vector3f &t);
    static void poseVecToRandT(const Vec<float> &pose, Eigen::Matrix2f &R, Eigen::Vector2f &t);
    static Eigen::Matrix4f poseVecToMat(const Vec<float> &pose);
    static Vec<float> poseMatToVec(const Eigen::Matrix4f &pose);
    static Vec<float> degToRad(const Vec<float> deg);
    static float degToRad(float deg);

    static Eigen::VectorXf VecToEigen(const Vec<float> &vec);
    static Eigen::VectorXf VecToEigen(const Vec<PQP_REAL> &vec);
    static Vec<float> EigenToVec(const Eigen::VectorXf &eigenVec);
};

} /* namespace rmpl */

#endif    // UTILITIES_H_
