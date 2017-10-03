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

#ifndef TRAJECTORYPLANNER_HPP
#define TRAJECTORYPLANNER_HPP

#include <ippp/core/Identifier.h>
#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/core/types.h>
#include <ippp/core/util/UtilTrajectory.hpp>
#include <ippp/environment/Environment.h>

namespace ippp {

/*!
* \brief   Class LinearTrajectory plans a path between the passed nodes/vecs. Start and end point aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class TrajectoryPlanner : public Identifier {
  public:
    TrajectoryPlanner(const std::string &name, const std::shared_ptr<CollisionDetection<dim>> &collision,
                      const std::shared_ptr<Environment> &environment, const double stepSize = 1);

    bool checkTrajectory(const Node<dim> &source, const Node<dim> &target);
    bool checkTrajectory(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    bool checkTrajectory(const Vector<dim> &source, const Vector<dim> &target);

    Vector<dim> checkTrajCont(const Node<dim> &source, const Node<dim> &target);
    Vector<dim> checkTrajCont(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    Vector<dim> checkTrajCont(const Vector<dim> &source, const Vector<dim> &target);

    virtual std::vector<Vector<dim>> calcTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target) = 0;
    virtual std::vector<Vector<dim>> calcTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target) = 0;

    void setStepSize(const double stepSize);
    double getStepSize() const;

  protected:
    double m_stepSize = 1;
    double m_sqStepSize = 1;
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
};

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  CollisionDetection
*  \param[in]  Environment
*  \param[in]  step size of the path
*  \date       2016-05-25
*/
template <unsigned int dim>
TrajectoryPlanner<dim>::TrajectoryPlanner(const std::string &name, const std::shared_ptr<CollisionDetection<dim>> &collision,
                                          const std::shared_ptr<Environment> &environment, const double stepSize)
    : Identifier(name), m_collision(collision), m_environment(environment) {
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
bool TrajectoryPlanner<dim>::checkTrajectory(const Node<dim> &source, const Node<dim> &target) {
    return checkTrajectory(source.getValues(), target.getValues());
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
bool TrajectoryPlanner<dim>::checkTrajectory(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target) {
    return checkTrajectory(source->getValues(), target->getValues());
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
template <unsigned int dim>
bool TrajectoryPlanner<dim>::checkTrajectory(const Vector<dim> &source, const Vector<dim> &target) {
    auto path = calcTrajectoryBin(source, target);
    if (m_collision->checkTrajectory(path))
        return false;

    return true;
}

/*!
*  \brief      Control the linear trajectory and return the last collision valid point.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] last collision valid point (NaN Vector, if no point is valid)
*  \date       2016-05-31
*/
template <unsigned int dim>
Vector<dim> TrajectoryPlanner<dim>::checkTrajCont(const Node<dim> &source, const Node<dim> &target) {
    return checkTrajCont(source.getValues(), target.getValues());
}

/*!
*  \brief      Control the linear trajectory and return the last collision valid point.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] last collision valid point (NaN Vector, if no point is valid)
*  \date       2016-05-31
*/
template <unsigned int dim>
Vector<dim> TrajectoryPlanner<dim>::checkTrajCont(const std::shared_ptr<Node<dim>> &source,
                                                  const std::shared_ptr<Node<dim>> &target) {
    return checkTrajCont(source->getValues(), target->getValues());
}

/*!
*  \brief      Control the linear trajectory and return the last collision valid point.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] last collision valid point (NaN Vector, if no point is valid)
*  \date       2016-05-31
*/
template <unsigned int dim>
Vector<dim> TrajectoryPlanner<dim>::checkTrajCont(const Vector<dim> &source, const Vector<dim> &target) {
    auto path = calcTrajectoryCont(source, target);
    path.push_back(target);
    unsigned int count = -1;
    for (auto point : path)
        if (m_collision->checkConfig(point))
            break;
        else
            ++count;

    if (count == -1)
        return util::NaNVector<dim>();
    else
        return path[count];
}

/*!
*  \brief      Set step size, if value is larger than zero otherwise 1 will be set
*  \author     Sascha Kaden
*  \param[in]  step size
*  \date       2016-07-14
*/
template <unsigned int dim>
void TrajectoryPlanner<dim>::setStepSize(const double stepSize) {
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
double TrajectoryPlanner<dim>::getStepSize() const {
    return m_stepSize;
}

} /* namespace ippp */

#endif /* TRAJECTORYPLANNER_HPP */
