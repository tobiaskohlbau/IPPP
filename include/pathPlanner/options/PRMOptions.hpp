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

#ifndef PRMOPTIONS_HPP
#define PRMOPTIONS_HPP

#include <pathPlanner/options/PlannerOptions.hpp>

namespace ippp {

/*!
* \brief   Class PRMOptions determines special options for the Pipppanner
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class PRMOptions : public PlannerOptions<dim> {
  public:
    PRMOptions(const float rangeSize, const std::shared_ptr<CollisionDetection<dim>> &collision,
               const std::shared_ptr<TrajectoryPlanner<dim>> &planner, const std::shared_ptr<Sampling<dim>> &sampling,
               const std::shared_ptr<Heuristic<dim>> &heuristic = std::shared_ptr<Heuristic<dim>>(new Heuristic<dim>()),
               const unsigned int sortingCountGraph = 0);

    void setRangeSize(const float rangeSize);
    float getRangeSize() const;

  private:
    float m_rangeSize = 30;
};

/*!
*  \brief      Standard constructor of the class PRMOptions
*  \param[in]  rangeSize
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
PRMOptions<dim>::PRMOptions(const float rangeSize, const std::shared_ptr<CollisionDetection<dim>> &collision, const std::shared_ptr<TrajectoryPlanner<dim>> &planner,
                            const std::shared_ptr<Sampling<dim>> &sampling,
                            const std::shared_ptr<Heuristic<dim>> &heuristic, const unsigned int sortingCountGraph)
    : PlannerOptions<dim>(collision, planner, sampling, heuristic, sortingCountGraph) {
    setRangeSize(rangeSize);
}

/*!
*  \brief      Sets the range size of the local planner from the Pipppanner
*  \param[in]  rangeSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PRMOptions<dim>::setRangeSize(const float rangeSize) {
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
float PRMOptions<dim>::getRangeSize() const {
    return m_rangeSize;
}

} /* namespace ippp */

#endif    // PRMOPTIONS_HPP
