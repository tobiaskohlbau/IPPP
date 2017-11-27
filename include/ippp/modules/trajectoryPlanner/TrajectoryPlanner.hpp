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

#include <ippp/Identifier.h>
#include <ippp/environment/Environment.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/types.h>
#include <ippp/util/UtilTrajectory.hpp>
#include <utility>

namespace ippp {

/*!
* \brief   Class LinearTrajectory plans a path between the passed nodes/configs. Start and end point aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class TrajectoryPlanner : public Identifier {
  public:
    TrajectoryPlanner(const std::string &name, std::shared_ptr<CollisionDetection<dim>> collision,
                      const std::shared_ptr<Environment> &environment, double posRes = 1, double oriRes = 0.1);

    bool checkTrajectory(const Node<dim> &source, const Node<dim> &target);
    bool checkTrajectory(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    bool checkTrajectory(const Vector<dim> &source, const Vector<dim> &target);

    Vector<dim> checkTrajCont(const Node<dim> &source, const Node<dim> &target);
    Vector<dim> checkTrajCont(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    Vector<dim> checkTrajCont(const Vector<dim> &source, const Vector<dim> &target);

    virtual std::vector<Vector<dim>> calcTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target) = 0;
    virtual std::vector<Vector<dim>> calcTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target) = 0;

    void setResolutions(double posRes, double oriRes = 0.1);
    double getPosRes() const;
    double getOriRes() const;
    std::pair<double, double> getResolutions() const;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;

    double m_posRes = 1;
    double m_oriRes = 0.1;
    double m_sqPosRes = 1;
    double m_sqOriRes = 1;

    Vector<dim> m_posMask;
    Vector<dim> m_oriMask;
};

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  CollisionDetection
*  \param[in]  Environment
*  \param[in]  position resolution
*  \param[in]  orientation resolution
*  \date       2016-05-25
*/
template <unsigned int dim>
TrajectoryPlanner<dim>::TrajectoryPlanner(const std::string &name, const std::shared_ptr<CollisionDetection<dim>> &collision,
                                          const std::shared_ptr<Environment> &environment, const double posRes,
                                          const double oriRes)
    : Identifier(name), m_collision(std::move(collision)), m_environment(environment) {
    Logging::debug("Initialize", this);

    setResolutions(posRes, oriRes);
    auto masks = environment->getConfigMasks();
    m_posMask = masks.first;
    m_oriMask = masks.second;
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
    !m_collision->checkTrajectory(path);
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
*  \brief      Set position and orientation resolutions.
*  \details    If values are smaller or equal to zero, standard parameter will be set.
*  \author     Sascha Kaden
*  \param[in]  position resolution
*  \param[in]  orientation resolution
*  \date       2017-10-07
*/
template <unsigned int dim>
void TrajectoryPlanner<dim>::setResolutions(const double posRes, const double oriRes) {
    if (posRes <= 0) {
        m_posRes = 1;
        Logging::warning("Position resolution has to be larger than 0, it was set to 1!", this);
    } else {
        m_posRes = posRes;
    }
    if (oriRes <= 0) {
        m_oriRes = 0.1;
        Logging::warning("Orientation resolution has to be larger than 0, it was set to 0.1!", this);
    } else {
        m_oriRes = oriRes;
    }

    m_sqPosRes = m_posRes * m_posRes;
    m_sqOriRes = m_oriRes * m_oriRes;
}

/*!
*  \brief      Return position resolution
*  \author     Sascha Kaden
*  \param[out] position resolution
*  \date       2017-10-07
*/
template <unsigned int dim>
double TrajectoryPlanner<dim>::getPosRes() const {
    return m_posRes;
}

/*!
*  \brief      Return orientation resolution
*  \author     Sascha Kaden
*  \param[out] orientation resolution
*  \date       2017-10-07
*/
template <unsigned int dim>
double TrajectoryPlanner<dim>::getOriRes() const {
    return m_oriRes;
}

/*!
*  \brief      Return orientation resolution
*  \author     Sascha Kaden
*  \param[out] orientation resolution
*  \date       2017-10-07
*/
template <unsigned int dim>
std::pair<double, double> TrajectoryPlanner<dim>::getResolutions() const {
    return std::make_pair(m_posRes, m_oriRes);
}

} /* namespace ippp */

#endif /* TRAJECTORYPLANNER_HPP */
