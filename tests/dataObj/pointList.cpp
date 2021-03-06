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

#include <gtest/gtest.h>

#include <ippp/dataObj/PointList.hpp>

using namespace ippp;

TEST(POINTLIST, standardConstructor2D) {
    std::vector<Vector2> vecs;
    vecs.reserve(4);
    for (int i = 0; i < 4; ++i)
        vecs.emplace_back(i, i);
    Line2D line(vecs[0], vecs[1]);
    Triangle2D triangle(vecs[0], vecs[1], vecs[2]);
    Square2D square(vecs[0], vecs[1], vecs[2], vecs[3]);

    for (size_t i = 0; i < 2; ++i)
        EXPECT_EQ(line.getP(i + 1), vecs[i]);
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(triangle.getP(i + 1), vecs[i]);
    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(square.getP(i + 1), vecs[i]);
}

TEST(POINTLIST, standardConstructor3D) {
    std::vector<Vector3> vecs;
    vecs.reserve(4);
    for (int i = 0; i < 4; ++i)
        vecs.emplace_back(i, i, i);
    Line3D line(vecs[0], vecs[1]);
    Triangle3D triangle(vecs[0], vecs[1], vecs[2]);
    Square3D square(vecs[0], vecs[1], vecs[2], vecs[3]);

    for (size_t i = 0; i < 2; ++i)
        EXPECT_EQ(line.getP(i + 1), vecs[i]);
    for (size_t i = 0; i < 3; ++i)
        EXPECT_EQ(triangle.getP(i + 1), vecs[i]);
    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(square.getP(i + 1), vecs[i]);
}
