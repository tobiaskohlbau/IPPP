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

#ifndef PRMOPTIONS_HPP
#define PRMOPTIONS_HPP

#include <ippp/motionPlanner/options/MPOptions.hpp>

namespace ippp {

/*!
* \brief   Class PRMOptions determines special options for the MotionPlanner PRM.
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class PRMOptions : public MPOptions<dim> {
  public:
    PRMOptions(double rangeSize, const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
               const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Evaluator<dim>> &evaluator,
               const std::shared_ptr<PathModifier<dim>> &pathModifier, const std::shared_ptr<Sampling<dim>> &sampling,
               const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    void setRangeSize(double rangeSize);
    double getRangeSize() const;

  private:
    double m_rangeSize = 30;
};

/*!
*  \brief      Standard constructor of the class PRMOptions
*  \param[in]  rangeSize
*  \param[in]  CollisionDetection
*  \param[in]  DistanceMetric
*  \param[in]  Evaluator
*  \param[in]  PathModifier
*  \param[in]  Sampling
*  \param[in]  TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
PRMOptions<dim>::PRMOptions(double rangeSize, const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                            const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Evaluator<dim>> &evaluator,
                            const std::shared_ptr<PathModifier<dim>> &pathModifier,
                            const std::shared_ptr<Sampling<dim>> &sampling,
                            const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : MPOptions<dim>(validityChecker, metric, evaluator, pathModifier, sampling, trajectory) {
    setRangeSize(rangeSize);
}

/*!
*  \brief      Sets the range size of the local planner from the Pipppanner
*  \param[in]  rangeSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PRMOptions<dim>::setRangeSize(double rangeSize) {
    if (rangeSize <= 0) {
        Logging::warning("Step size was equal or smaller than 0 and is set up to 1", this);
        m_rangeSize = 1;
    } else {
        m_rangeSize = rangeSize;
    }
}

/*!
*  \brief      Returns the range size of the local planner from the Pipppanner
*  \param[out] rangeSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
double PRMOptions<dim>::getRangeSize() const {
    return m_rangeSize;
}

} /* namespace ippp */

#endif    // PRMOPTIONS_HPP
