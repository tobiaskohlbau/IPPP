//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#ifndef LINEARTRAJECTORY_HPP
#define LINEARTRAJECTORY_HPP

#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {

/*!
* \brief   Class LinearTrajectory plans a linear discrete trajectory between the passed nodes/configs.
* \details Start and end configuration aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class LinearTrajectory : public TrajectoryPlanner<dim> {
  public:
    LinearTrajectory(const std::shared_ptr<Environment> &environment, double posRes = 1, double oriRes = util::toRad(5));

    std::vector<Vector<dim>> calcTrajCont(const Vector<dim> &source, const Vector<dim> &target) const;
    std::vector<Vector<dim>> calcTrajBin(const Vector<dim> &source, const Vector<dim> &target) const;

  private:
    using TrajectoryPlanner<dim>::m_posRes;
    using TrajectoryPlanner<dim>::m_oriRes;
    using TrajectoryPlanner<dim>::m_sqPosRes;
    using TrajectoryPlanner<dim>::m_sqOriRes;
    using TrajectoryPlanner<dim>::m_posMask;
    using TrajectoryPlanner<dim>::m_oriMask;
};

/*!
*  \brief      Constructor of the class LinearTrajectory
*  \author     Sascha Kaden
*  \param[in]  CollisionDetection
*  \param[in]  Environment
*  \param[in]  position resolution
*  \param[in]  orientation resolution
*  \date       2016-05-25
*/
template <unsigned int dim>
LinearTrajectory<dim>::LinearTrajectory(const std::shared_ptr<Environment> &environment, double posRes, double oriRes)
    : TrajectoryPlanner<dim>("LinearTrajectory", environment, posRes, oriRes) {
}

/*!
*  \brief      Compute the binary (section wise) trajectory between source and target. Return vector of configs.
*  \details    The trajectory doesn't contain source or target vector.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] discrete trajectory of configs
*  \date       2016-12-21
*/
template <unsigned int dim>
std::vector<Vector<dim>> LinearTrajectory<dim>::calcTrajBin(const Vector<dim> &source, const Vector<dim> &target) const {
    return util::linearTrajectoryBin<dim>(source, target, m_posRes, m_oriRes, m_posMask, m_oriMask);
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of configs.
*  \details    The trajectory doesn't contain source or target vector.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] discrete trajectory of configs
*  \date       2016-12-21
*/
template <unsigned int dim>
std::vector<Vector<dim>> LinearTrajectory<dim>::calcTrajCont(const Vector<dim> &source, const Vector<dim> &target) const {
    return util::linearTrajectoryCont<dim>(source, target, m_posRes, m_oriRes, m_posMask, m_oriMask);
}

} /* namespace ippp */

#endif /* LINEARTRAJECTORY_HPP */
