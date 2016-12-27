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

#include <iostream>

#include <boost/test/unit_test.hpp>

#include <core/utility/Utility.h>

using namespace rmpl;
using namespace Eigen;

BOOST_AUTO_TEST_SUITE(staticFunctions)

BOOST_AUTO_TEST_CASE(pi) {
    BOOST_CHECK(utilGeo::pi() < 3.141593);
    BOOST_CHECK(utilGeo::pi() > 3.141591);
}

BOOST_AUTO_TEST_CASE(decomposeT) {
    Matrix4f T0 = Matrix4f::Zero(4, 4);
    Matrix4f T1 = Matrix4f::Identity(4, 4);
    Matrix3f R0, R1;
    Vector3f t0, t1;
    utilGeo::decomposeT(T0, R0, t0);
    utilGeo::decomposeT(T1, R1, t1);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            BOOST_CHECK(R0(i, j) == 0);
            if (i != j)
                BOOST_CHECK(R1(i, j) == 0);
            else
                BOOST_CHECK(R1(i, j) == 1);
        }
        BOOST_CHECK(t0(i, 0) == 0);
        BOOST_CHECK(t1(i, 0) == 0);
    }
}

BOOST_AUTO_TEST_CASE(poseVecToMat) {
    Eigen::Matrix<float, 6, 1> poseZero = utilVec::Vecf(0, 0, 0, 0, 0, 0);
    Matrix4f poseMat = utilGeo::poseVecToMat(poseZero);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                BOOST_CHECK(poseMat(i, j) == 1);
            } else {
                BOOST_CHECK(poseMat(i, j) == 0);
            }
        }
    }
    Eigen::Matrix<float, 6, 1> poseVec = utilGeo::poseMatToVec(poseMat);
    for (int i = 0; i < 6; ++i)
        if (poseVec[i] < 0.0001 && poseVec[i] > -0.0001)
            poseVec[i] = 0;
    for (int i = 0; i < 6; ++i)
        BOOST_CHECK(poseZero[i] == poseVec[i]);

    Eigen::Matrix<float, 6, 1> poseTwo = utilVec::Vecf(1, 1, 1, 0, 0, 0);
    poseMat = utilGeo::poseVecToMat(poseTwo);
    for (int i = 0; i < 4; ++i)
        BOOST_CHECK(poseMat(i, 3) == 1);

    for (float x = 0; x < 359; x += 5) {
        for (float y = 0; y < 359; y += 5) {
            for (float z = 0; z < 359; z += 5) {
                poseMat = utilGeo::poseVecToMat(poseTwo);
                poseVec = utilGeo::poseMatToVec(poseMat);
                for (int i = 0; i < 6; ++i) {
                    BOOST_CHECK(poseTwo[i] <= poseVec[i] + 0.0001);
                    BOOST_CHECK(poseTwo[i] >= poseVec[i] - 0.0001);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(degToRad) {
    float pi = utilGeo::pi();
    float a1[11] = {0, 30, 45, 60, 90, 120, 135, 150, 180, 270, 360};
    float a2[11] = {0, pi / 6, pi / 4, pi / 3, pi / 2, 2 * pi / 3, 3 * pi / 4, 5 * pi / 6, pi, 3 * pi / 2, 2 * pi};
    Eigen::VectorXf deg = utilVec::Vecf(11, a1);
    Eigen::VectorXf rad = utilVec::Vecf(11, a2);
    Eigen::VectorXf temp = utilGeo::degToRad(deg);
    for (int i = 0; i < deg.rows(); ++i) {
        BOOST_CHECK(rad[i] <= temp[i] + 0.0001);
        BOOST_CHECK(rad[i] >= temp[i] - 0.0001);
    }
}

BOOST_AUTO_TEST_SUITE_END()
