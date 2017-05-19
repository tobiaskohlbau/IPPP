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

#ifndef COLLISIONDETECTION2D_H
#define COLLISIONDETECTION2D_H

#include <core/collisionDetection/CollisionDetection.hpp>
#include <environment/CadProcessing.h>
#include <environment/model/ModelTriangle2D.h>
#include <environment/robot/PointRobot.h>

namespace ippp {

/*!
* \brief   Class for 2D collision detection of an point robot.
* \author  Sascha Kaden
* \date    2017-02-19
*/
class CollisionDetection2D : public CollisionDetection<2> {
  public:
    CollisionDetection2D(const std::shared_ptr<Environment> &environment);
    bool controlVec(const Vector2 &vec) override;
    bool controlTrajectory(std::vector<Vector2> &vec) override;

  private:
    bool checkPoint2D(float x, float y);

    Vector2 m_minBoundary;
    Vector2 m_maxBoundary;
    Eigen::MatrixXi m_workspace2D;
};

} /* namespace ippp */

#endif /* COLLISIONDETECTION2D_H */
