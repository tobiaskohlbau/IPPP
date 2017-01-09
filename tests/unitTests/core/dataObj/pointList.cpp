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

#include <core/dataObj/PointList.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(standardConstructor2D) {
    std::vector<Vector2> vecs;
    for (int i = 0; i < 4; ++i)
        vecs.push_back(Vector2(i, i));
    Line2D line(vecs[0], vecs[1]);
    Triangle2D triangle(vecs[0], vecs[1], vecs[2]);
    Square2D square(vecs[0], vecs[1], vecs[2], vecs[3]);

    BOOST_CHECK(line.getP(1) == vecs[0]);
    BOOST_CHECK(line.getP(2) == vecs[1]);
    BOOST_CHECK(triangle.getP(1) == vecs[0]);
    BOOST_CHECK(triangle.getP(2) == vecs[1]);
    BOOST_CHECK(triangle.getP(3) == vecs[2]);
    BOOST_CHECK(square.getP(1) == vecs[0]);
    BOOST_CHECK(square.getP(2) == vecs[1]);
    BOOST_CHECK(square.getP(3) == vecs[2]);
    BOOST_CHECK(square.getP(4) == vecs[3]);
}

BOOST_AUTO_TEST_CASE(standardConstructor3D) {
    std::vector<Vector3> vecs;
    for (int i = 0; i < 4; ++i)
        vecs.push_back(Vector3(i, i, i));
    Line3D line(vecs[0], vecs[1]);
    Triangle3D triangle(vecs[0], vecs[1], vecs[2]);
    Square3D square(vecs[0], vecs[1], vecs[2], vecs[3]);

    BOOST_CHECK(line.getP(1) == vecs[0]);
    BOOST_CHECK(line.getP(2) == vecs[1]);
    BOOST_CHECK(triangle.getP(1) == vecs[0]);
    BOOST_CHECK(triangle.getP(2) == vecs[1]);
    BOOST_CHECK(triangle.getP(3) == vecs[2]);
    BOOST_CHECK(square.getP(1) == vecs[0]);
    BOOST_CHECK(square.getP(2) == vecs[1]);
    BOOST_CHECK(square.getP(3) == vecs[2]);
    BOOST_CHECK(square.getP(4) == vecs[3]);
}

BOOST_AUTO_TEST_SUITE_END()
