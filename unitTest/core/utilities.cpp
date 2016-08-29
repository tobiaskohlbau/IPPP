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

#include <boost/test/unit_test.hpp>

#include <core/Utilities.h>

using namespace rmpl;
using namespace Eigen;

BOOST_AUTO_TEST_SUITE(staticFunctions)

BOOST_AUTO_TEST_CASE(pi) {
    BOOST_CHECK(Utilities::pi() < 3.141593);
    BOOST_CHECK(Utilities::pi() > 3.141591);
}

BOOST_AUTO_TEST_CASE(decomposeT) {
    Matrix4f T0 = Matrix4f::Zero(4, 4);
    Matrix4f T1 = Matrix4f::Identity(4, 4);
    Matrix3f R0, R1;
    Vector3f t0, t1;
    Utilities::decomposeT(T0, R0, t0);
    Utilities::decomposeT(T1, R1, t1);

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
    Vec<float> poseZero(0, 0, 0, 0, 0, 0);
    Matrix4f poseMat = Utilities::poseVecToMat(poseZero);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                BOOST_CHECK(poseMat(i, j) == 1);
            } else {
                BOOST_CHECK(poseMat(i, j) == 0);
            }
        }
    }
    Vec<float> poseVec = Utilities::poseMatToVec(poseMat);
    for (int i = 0; i < 6; ++i)
        BOOST_CHECK(poseZero[i] == poseVec[i]);

    Vec<float> poseTwo(1, 1, 1, 0, 0, 0);
    poseMat = Utilities::poseVecToMat(poseTwo);
    for (int i = 0; i < 4; ++i)
        BOOST_CHECK(poseMat(i, 3) == 1);

    for (float x = 0; x < 359; x += 5) {
        for (float y = 0; y < 359; y += 5) {
            for (float z = 0; z < 359; z += 5) {
                poseMat = Utilities::poseVecToMat(poseTwo);
                poseVec = Utilities::poseMatToVec(poseMat);
                for (int i = 0; i < 6; ++i) {
                    BOOST_CHECK(poseTwo[i] <= poseVec[i] + 0.0001);
                    BOOST_CHECK(poseTwo[i] >= poseVec[i] - 0.0001);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(degToRad) {
    float pi = Utilities::pi();
    float a1[11] = {0, 30, 45, 60, 90, 120, 135, 150, 180, 270, 360};
    float a2[11] = {0, pi / 6, pi / 4, pi / 3, pi / 2, 2 * pi / 3, 3 * pi / 4, 5 * pi / 6, pi, 3 * pi / 2, 2 * pi};
    Vec<float> deg(11, a1);
    Vec<float> rad(11, a2);
    Vec<float> temp = Utilities::degToRad(deg);
    for (int i = 0; i < deg.getDim(); ++i) {
        BOOST_CHECK(rad[i] <= temp[i] + 0.0001);
        BOOST_CHECK(rad[i] >= temp[i] - 0.0001);
    }
}

BOOST_AUTO_TEST_CASE(VecToEigen) {
    for (float i = 0; i < 7; i += 0.3) {
        for (float j = 0; j < 3; j += 0.2) {
            for (float k = 0; k < 3; k += 0.123) {
                Vec<float> vec(i, j, k);
                ArrayXf array = Utilities::VecToEigen(vec);
                for (int i = 0; i < vec.getDim(); ++i) {
                    BOOST_CHECK(vec[i] <= array(i, 0) + 0.0001);
                    BOOST_CHECK(vec[i] >= array(i, 0) - 0.0001);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(EigenToVec) {
    for (float i = 0; i < 7; i += 0.3) {
        for (float j = 0; j < 3; j += 0.2) {
            for (float k = 0; k < 3; k += 0.123) {
                Array3f array;
                array(0, 0) = i;
                array(1, 0) = j;
                array(2, 0) = k;
                Vec<float> vec = Utilities::EigenToVec(array);
                for (int i = 0; i < vec.getDim(); ++i) {
                    BOOST_CHECK(array(i, 0) <= vec[i] + 0.0001);
                    BOOST_CHECK(array(i, 0) >= vec[i] - 0.0001);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
