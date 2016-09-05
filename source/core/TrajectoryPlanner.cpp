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

#include <core/TrajectoryPlanner.h>

#include <core/Logging.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  TrajectoryMethod
*  \param[in]  pointer to ColllisionDetection instance
*  \date       2016-05-25
*/
TrajectoryPlanner::TrajectoryPlanner(const TrajectoryMethod &method, float stepSize,
                                     const shared_ptr<CollisionDetection> &collision)
    : ModuleBase("TrajectoryPlanner") {
    m_method = method;
    m_collision = collision;

    if (stepSize <= 0) {
        Logging::warning("Step size has to be larger than 0, it has set to 0.1!", this);
        m_stepSize = 0.1;
    } else {
        m_stepSize = stepSize;
    }
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const Node &source, const Node &target) {
    return controlTrajectory(source.getVec(), target.getVec());
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target) {
    return controlTrajectory(source->getVec(), target->getVec());
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  source Vec
*  \param[in]  target Vec
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const Vec<float> &source, const Vec<float> &target) {
    if (source.getDim() != target.getDim()) {
        Logging::error("Nodes/Vecs have different dimensions", this);
        return false;
    } else if (m_stepSize == -1) {
        Logging::error("Step size is not set!", this);
        return false;
    }

    std::vector<Vec<float>> path = computeTrajectory(source, target);
    if (m_collision->controlTrajectory(path))
        return false;

    return true;
}

/*!
*  \brief      Compute the trajectory and return the list of vecs
*  \author     Sascha Kaden
*  \param[in]  first Vec
*  \param[in]  second Vec
*  \param[in]  step size
*  \param[out] trajectory
*  \date       2016-05-31
*/
std::vector<Vec<float>> TrajectoryPlanner::computeTrajectory(const Vec<float> &source, const Vec<float> &target, float stepSize) {
    std::vector<Vec<float>> vecs;

    if (source.getDim() != target.getDim()) {
        Logging::error("Nodes/Vecs have different dimensions", this);
        return vecs;
    } else if (m_stepSize == -1) {
        Logging::error("Step size is not set!", this);
        return vecs;
    }

    if (stepSize == 0)
        stepSize = m_stepSize;

    Vec<float> u = target - source;
    Vec<float> uNorm = u / u.norm();
    Vec<float> temp(source);
    while ((temp - target).sqNorm() > 0.5) {
        vecs.push_back(temp);
        temp += uNorm * stepSize;
    }
    return vecs;
}

/*!
*  \brief      Set step size, if it is larger than zero
*  \author     Sascha Kaden
*  \param[in]  step size
*  \date       2016-07-14
*/
void TrajectoryPlanner::setStepSize(float stepSize) {
    if (stepSize <= 0) {
        m_stepSize = 0.1;
        Logging::warning("Step size has to be larger than 0, it has set to 0.1!", this);
    }
    else {
        m_stepSize = stepSize;}
}

/*!
*  \brief      Return step size
*  \author     Sascha Kaden
*  \param[out] step size
*  \date       2016-07-14
*/
float TrajectoryPlanner::getStepSize() {
    return m_stepSize;
}
