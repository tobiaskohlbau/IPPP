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

#include <robot/TriangleRobot.h>

#include <core/Logging.h>

using namespace rmpl;

/*!
*  \brief      Standard constructor of the 2D TriangleRobot
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
TriangleRobot::TriangleRobot(Vec<float> minBoundary, Vec<float> maxBoundary)
    : MobileRobot("TriangleRobot", CollisionType::triangle2D, 3, minBoundary, maxBoundary) {
}

/*!
*  \brief      Set vector of the triangles
*  \param[in]  vector of triangles
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void TriangleRobot::setTriangles(std::vector<Triangle> triangles) {
    m_triangles = triangles;
}

/*!
*  \brief      Return vector of the triangles
*  \param[out] vector of triangles
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
std::vector<Triangle> TriangleRobot::getTriangles() {
    return m_triangles;
}
