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
#include <ippp/dataObj/Node.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/types.h>
#include <ippp/util/UtilTrajectory.hpp>

namespace ippp {

/*!
* \brief   Class LinearTrajectory plans a path between the passed nodes/configs. Start and end point aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class TrajectoryPlanner : public Identifier {
  public:
    TrajectoryPlanner(const std::string &name, const std::shared_ptr<Environment> &environment, double posRes = 1,
                      double oriRes = 0.1);

    std::vector<Vector<dim>> calcTrajCont(const Node<dim> &source, const Node<dim> &target) const;
    std::vector<Vector<dim>> calcTrajBin(const Node<dim> &source, const Node<dim> &target) const;
    virtual std::vector<Vector<dim>> calcTrajCont(const Vector<dim> &source, const Vector<dim> &target) const = 0;
    virtual std::vector<Vector<dim>> calcTrajBin(const Vector<dim> &source, const Vector<dim> &target) const = 0;

    void setResolutions(double posRes, double oriRes = 0.1);
    void setResolutions(std::pair<double, double> resolutions);
    std::pair<double, double> getResolutions() const;

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;

    double m_posRes = 1;
    double m_oriRes = 0.1;
    double m_sqPosRes = 1;
    double m_sqOriRes = 0.01;

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
TrajectoryPlanner<dim>::TrajectoryPlanner(const std::string &name, const std::shared_ptr<Environment> &environment, double posRes,
                                          double oriRes)
    : Identifier(name), m_environment(environment) {
    Logging::debug("Initialize", this);

    setResolutions(posRes, oriRes);
    auto masks = environment->getConfigMasks();
    m_posMask = masks.first;
    m_oriMask = masks.second;
}

template <unsigned int dim>
std::vector<Vector<dim>> TrajectoryPlanner<dim>::calcTrajCont(const Node<dim> &source, const Node<dim> &target) const {
    return calcTrajCont(source.getValues(), target.getValues());
}
template <unsigned int dim>
std::vector<Vector<dim>> TrajectoryPlanner<dim>::calcTrajBin(const Node<dim> &source, const Node<dim> &target) const {
    return calcTrajBin(source.getValues(), target.getValues());
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
void TrajectoryPlanner<dim>::setResolutions(double posRes, double oriRes) {
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
*  \brief      Set position and orientation resolutions.
*  \details    If values are smaller or equal to zero, standard parameter will be set.
*  \author     Sascha Kaden
*  \param[in]  position and orientation resolution
*  \date       2018-02-20
*/
template <unsigned int dim>
void TrajectoryPlanner<dim>::setResolutions(std::pair<double, double> resolutions) {
    setResolutions(resolutions.first, resolutions.second);
}

/*!
*  \brief      Return the resolutions, pair of position and orientation resolution
*  \author     Sascha Kaden
*  \param[out] position and orientation resolution
*  \date       2018-02-20
*/
template <unsigned int dim>
std::pair<double, double> TrajectoryPlanner<dim>::getResolutions() const {
    return std::make_pair(m_posRes, m_oriRes);
}

} /* namespace ippp */

#endif /* TRAJECTORYPLANNER_HPP */
