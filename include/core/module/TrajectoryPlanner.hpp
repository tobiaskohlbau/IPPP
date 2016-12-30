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

#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include <core/module/CollisionDetection.hpp>
#include <core/module/ModuleBase.h>

namespace rmpl {

/*!
* \brief   Class TrajectoryPlanner plans between the passed nodes/vecs. Start and end point aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class TrajectoryPlanner : public ModuleBase {
  public:
    TrajectoryPlanner(float stepSize, const std::shared_ptr<CollisionDetection<dim>> &collision);

    bool controlTrajectory(const Node<dim> &source, const Node<dim> &target);
    bool controlTrajectory(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    bool controlTrajectory(const Eigen::VectorXf &source, const Eigen::VectorXf &target);
    std::vector<Eigen::VectorXf> calcTrajectoryCont(const Eigen::VectorXf &source, const Eigen::VectorXf &target);
    std::vector<Eigen::VectorXf> calcTrajectoryBin(const Eigen::VectorXf &source, const Eigen::VectorXf &target);

    void setStepSize(float stepSize);
    float getStepSize() const;

  private:
    float m_stepSize;
    float m_sqStepSize;
    std::shared_ptr<CollisionDetection<dim>> m_collision;
};

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  TrajectoryMethod
*  \param[in]  pointer to ColllisionDetection instance
*  \date       2016-05-25
*/
template <unsigned int dim>
TrajectoryPlanner<dim>::TrajectoryPlanner(float stepSize, const std::shared_ptr<CollisionDetection<dim>> &collision)
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
template <unsigned int dim>
bool TrajectoryPlanner<dim>::controlTrajectory(const Node<dim> &source, const Node<dim> &target) {
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
template <unsigned int dim>
bool TrajectoryPlanner<dim>::controlTrajectory(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target) {
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
template <unsigned int dim>
bool TrajectoryPlanner<dim>::controlTrajectory(const Eigen::VectorXf &source, const Eigen::VectorXf &target) {
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
template <unsigned int dim>
std::vector<Eigen::VectorXf> TrajectoryPlanner<dim>::calcTrajectoryBin(const Eigen::VectorXf &source,
                                                                       const Eigen::VectorXf &target) {
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
template <unsigned int dim>
std::vector<Eigen::VectorXf> TrajectoryPlanner<dim>::calcTrajectoryCont(const Eigen::VectorXf &source,
                                                                        const Eigen::VectorXf &target) {
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
template <unsigned int dim>
void TrajectoryPlanner<dim>::setStepSize(float stepSize) {
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
template <unsigned int dim>
float TrajectoryPlanner<dim>::getStepSize() const {
    return m_stepSize;
}

} /* namespace rmpl */

#endif /* TRAJECTORYPLANNER_H_ */
