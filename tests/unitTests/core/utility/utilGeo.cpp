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

#include <iostream>

#include <gtest/gtest.h>

#include <ippp/util/UtilGeo.hpp>

using namespace ippp;
using namespace Eigen;

TEST(GEO, pi) {
    EXPECT_TRUE(util::pi() < 3.141593);
    EXPECT_TRUE(util::pi() > 3.141591);
}

TEST(GEO, poseVecToMat) {
    Vector6 poseZero = util::Vecd(0, 0, 0, 0, 0, 0);
    ippp::Transform pose = util::poseVecToTransform(poseZero);
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(pose.translation()(i), 0);

    Vector6 poseVec = util::transformToVec(pose);
    for (int i = 0; i < 6; ++i)
        if (poseVec[i] < ippp::EPSILON && poseVec[i] > -ippp::EPSILON)
            poseVec[i] = 0;
    for (int i = 0; i < 6; ++i)
        EXPECT_EQ(poseZero[i], poseVec[i]);

    Vector6 poseTwo = util::Vecd(1, 1, 1, 0, 0, 0);
    pose = util::poseVecToTransform(poseTwo);
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(pose.translation()(i), 1);

    for (double x = 0; x < 359; x += 5) {
        for (double y = 0; y < 359; y += 5) {
            for (double z = 0; z < 359; z += 5) {
                pose = util::poseVecToTransform(poseTwo);
                poseVec = util::transformToVec(pose);
                for (int i = 0; i < 6; ++i) {
                    EXPECT_TRUE(poseTwo[i] <= poseVec[i] + ippp::EPSILON);
                    EXPECT_TRUE(poseTwo[i] >= poseVec[i] - ippp::EPSILON);
                }
            }
        }
    }
}

TEST(GEO, degToRad) {
    double pi = util::pi();
    double a1[11] = {0, 30, 45, 60, 90, 120, 135, 150, 180, 270, 360};
    double a2[11] = {0, pi / 6, pi / 4, pi / 3, pi / 2, 2 * pi / 3, 3 * pi / 4, 5 * pi / 6, pi, 3 * pi / 2, 2 * pi};
    VectorX deg = util::Vecd(11, a1);
    VectorX rad = util::Vecd(11, a2);
    VectorX temp1 = util::degToRad<11>(deg);
    Vector<11> temp2;
    for (int i = 0; i < 11; ++i)
        temp2[i] = deg[i] * util::toRad();
    for (int i = 0; i < deg.rows(); ++i) {
        EXPECT_TRUE(rad[i] <= temp1[i] + ippp::EPSILON);
        EXPECT_TRUE(rad[i] >= temp1[i] - ippp::EPSILON);
        EXPECT_TRUE(rad[i] <= temp2[i] + ippp::EPSILON);
        EXPECT_TRUE(rad[i] >= temp2[i] - ippp::EPSILON);
    }
}

TEST(GEO, radToDeg) {
    double pi = util::pi();
    double a1[11] = {0, pi / 6, pi / 4, pi / 3, pi / 2, 2 * pi / 3, 3 * pi / 4, 5 * pi / 6, pi, 3 * pi / 2, 2 * pi};
    double a2[11] = {0, 30, 45, 60, 90, 120, 135, 150, 180, 270, 360};
    VectorX rad = util::Vecd(11, a1);
    VectorX deg = util::Vecd(11, a2);
    VectorX temp1 = util::radToDeg<11>(rad);
    Vector<11> temp2;
    for (int i = 0; i < 11; ++i)
        temp2[i] = rad[i] * util::toDeg();
    for (int i = 0; i < deg.rows(); ++i) {
        EXPECT_TRUE(deg[i] <= temp1[i] + ippp::EPSILON);
        EXPECT_TRUE(deg[i] >= temp1[i] - ippp::EPSILON);
        EXPECT_TRUE(deg[i] <= temp2[i] + ippp::EPSILON);
        EXPECT_TRUE(deg[i] >= temp2[i] - ippp::EPSILON);
    }
}
