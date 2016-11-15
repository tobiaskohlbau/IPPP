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

#ifndef TRIANGLEROBOT_H_
#define TRIANGLEROBOT_H_

#include <core/Triangle.h>
#include <robot/MobileRobot.h>

namespace rmpl {

/*!
* \brief   Class for the 2D trianlge robot
* \author  Sascha Kaden
* \date    2016-11-14
*/
class TriangleRobot : public MobileRobot {
  public:
    TriangleRobot(Vec<float> minBoundary, Vec<float> maxBoundary);
    void setTriangles(std::vector<Triangle> triangles);
    std::vector<Triangle> getTriangles();

  private:
    Vec<float> m_pose; // x,y,rot(z)
    std::vector<Triangle> m_triangles;
};

} /* namespace rmpl */

#endif /* TRIANGLEROBOT_H_ */
