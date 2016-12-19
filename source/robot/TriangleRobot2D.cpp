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

#include <robot/TriangleRobot2D.h>

#include <include/core/utility/Logging.h>

namespace rmpl {

/*!
*  \brief      Standard constructor of the 2D TriangleRobot
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
TriangleRobot2D::TriangleRobot2D(std::vector<Triangle2D> triangles, Vec<float> minBoundary, Vec<float> maxBoundary)
    : MobileRobot("TriangleRobot2D", CollisionType::triangle2D, 3, minBoundary, maxBoundary) {
    setTriangles(triangles);
}

/*!
*  \brief      Set vector of the triangles
*  \param[in]  vector of triangles
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
bool TriangleRobot2D::setTriangles(std::vector<Triangle2D> &triangles) {
    m_triangles = triangles;
    return true;
}

/*!
*  \brief      Add Triangle to list
*  \param[in]  Triangle
*  \author     Sascha Kaden
*  \date       2016-11-21
*/
bool TriangleRobot2D::addTriangle(Triangle2D &triangle) {
    m_triangles.push_back(triangle);
    return true;
}

/*!
*  \brief      Return vector of the triangles
*  \param[out] vector of triangles
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
std::vector<Triangle2D> TriangleRobot2D::getTriangles() {
    return m_triangles;
}

} /* namespace rmpl */


