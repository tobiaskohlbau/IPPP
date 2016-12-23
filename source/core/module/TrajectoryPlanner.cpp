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

#include <core/module/TrajectoryPlanner.h>

#include <core/utility/Logging.h>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  TrajectoryMethod
*  \param[in]  pointer to ColllisionDetection instance
*  \date       2016-05-25
*/
TrajectoryPlanner::TrajectoryPlanner(float stepSize, const shared_ptr<CollisionDetection> &collision)
    : ModuleBase("TrajectoryPlanner") {
    m_collision = collision;

    setStepSize(stepSize);
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
    return controlTrajectory(source.getValues(), target.getValues());
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
    return controlTrajectory(source->getValues(), target->getValues());
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  source Vec
*  \param[in]  target Vec
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const Eigen::VectorXf &source, const Eigen::VectorXf &target) {
    if (source.rows() != target.rows()) {
        Logging::error("Nodes/Vecs have different dimensions", this);
        return false;
    }

    std::vector<Eigen::VectorXf> path = calcTrajectoryBin(source, target);
    if (m_collision->controlTrajectory(path))
        return false;

    return true;
}

/*!
*  \brief      Compute the binary (section wise) trajectory between source and target. Return vector of points.
*  \author     Sascha Kaden
*  \param[in]  source Vec
*  \param[in]  target Vec
*  \param[out] trajectory
*  \date       2016-12-21
*/
std::vector<Eigen::VectorXf> TrajectoryPlanner::calcTrajectoryBin(const Eigen::VectorXf &source, const Eigen::VectorXf &target) {
    std::vector<Eigen::VectorXf> vecs;
    if (source.rows() != target.rows()) {
        Logging::error("Vecs have different dimensions", this);
        return vecs;
    }

    Eigen::VectorXf u(target - source);
    vecs.push_back(source);
    unsigned int divider = 2;
    for (Eigen::VectorXf uTemp(u / divider); uTemp.squaredNorm() > m_sqStepSize; divider *= 2, uTemp = u / divider)
        for (int i = 1; i < divider; i += 2)
            vecs.push_back(source + (uTemp * i));
    vecs.push_back(target);

    return vecs;
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \author     Sascha Kaden
*  \param[in]  source Vec
*  \param[in]  target Vec
*  \param[out] trajectory
*  \date       2016-12-21
*/
std::vector<Eigen::VectorXf> TrajectoryPlanner::calcTrajectoryCont(const Eigen::VectorXf &source, const Eigen::VectorXf &target) {
    std::vector<Eigen::VectorXf> vecs;
    if (source.rows() != target.rows()) {
        Logging::error("Vecs have different dimensions", this);
        return vecs;
    }

    Eigen::VectorXf u(target - source);    // u = a - b
    u /= u.norm();                         // u = |u|
    for (Eigen::VectorXf temp(source + (u * m_stepSize)); (temp - target).squaredNorm() > 1; temp = temp + (u * m_stepSize))
        vecs.push_back(temp);
    return vecs;
}

/*!
*  \brief      Set step size, if value is larger than zero
*  \author     Sascha Kaden
*  \param[in]  step size
*  \date       2016-07-14
*/
void TrajectoryPlanner::setStepSize(float stepSize) {
    if (stepSize <= 0) {
        m_stepSize = 1;
        Logging::warning("Step size has to be larger than 0, it has set to 1!", this);
    } else {
        m_stepSize = stepSize;
    }
    m_sqStepSize = stepSize * stepSize;
}

/*!
*  \brief      Return step size
*  \author     Sascha Kaden
*  \param[out] step size
*  \date       2016-07-14
*/
float TrajectoryPlanner::getStepSize() const {
    return m_stepSize;
}

} /* namespace rmpl */