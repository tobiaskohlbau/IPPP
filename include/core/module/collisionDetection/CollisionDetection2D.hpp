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

#ifndef COLLISIONDETECTION2D_HPP
#define COLLISIONDETECTION2D_HPP

#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <robot/PointRobot.h>
#include <robot/model/Model2D.h>

namespace ippp {

/*!
* \brief   Class for 2D collision detection of an point robot.
* \author  Sascha Kaden
* \date    2017-02-19
*/
class CollisionDetection2D : public CollisionDetection<2> {
  public:
    CollisionDetection2D(const std::shared_ptr<RobotBase<2>> &robot);
    bool controlVec(const Vector2 &vec) override;
    bool controlTrajectory(std::vector<Vector2> &vec) override;

  private:
    bool checkPoint2D(float x, float y);

    Eigen::MatrixXi m_workspace2D;
};

/*!
*  \brief      Constructor of the class CollisionDetection2D
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
CollisionDetection2D::CollisionDetection2D(const std::shared_ptr<RobotBase<2>> &robot)
    : CollisionDetection<2>("CollisionDetection2D", robot) {
    if (!m_robot->getWorkspace() || m_robot->getWorkspace()->empty()) {
        Logging::error("Empty workspace model", this);
        return;
    } else {
        m_workspace2D = std::dynamic_pointer_cast<Model2D>(robot->getWorkspace())->m_space;
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
bool CollisionDetection2D::controlVec(const Vector2 &vec) {
    return checkPoint2D(vec[0], vec[1]);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
bool CollisionDetection2D::controlTrajectory(std::vector<Vector2> &vecs) {
    if (vecs.size() == 0) {
        return false;
    }
    for (int i = 0; i < vecs.size(); ++i) {
        if (checkPoint2D(vecs[i][0], vecs[i][1])) {
            return true;
        }
    }
    return false;
}

/*!
*  \brief      Check for 2D point collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-06-30
*/
bool CollisionDetection2D::checkPoint2D(float x, float y) {
    if (m_minBoundary[0] >= x || x >= m_maxBoundary[0] || m_minBoundary[1] >= y || y >= m_maxBoundary[1]) {
        Logging::debug("Point out of workspace", this);
        return true;
    }

    if (m_workspace2D(y, x) < 80) {
        return true;
    } else {
        return false;
    }
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION2D_HPP */
