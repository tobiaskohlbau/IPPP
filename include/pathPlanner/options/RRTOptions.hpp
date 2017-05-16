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

#ifndef RRTOPTIONS_HPP
#define RRTOPTIONS_HPP

#include <pathPlanner/options/PlannerOptions.hpp>

namespace ippp {

/*!
* \brief   Class RRTOptions determines special options for the RRTPlanner
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class RRTOptions : public PlannerOptions<dim> {
  public:
    RRTOptions(const float stepSize, const std::shared_ptr<CollisionDetection<dim>> &collision,
               const std::shared_ptr<TrajectoryPlanner<dim>> &planner, const std::shared_ptr<Sampling<dim>> &sampling,
               const std::shared_ptr<DistanceMetric<dim>> &metric);

    void setStepSize(const float stepSize);
    float getStepSize() const;

  private:
    float m_stepSize = 30;
};

/*!
*  \brief      Standard constructor of the class RRTOptions
*  \param[in]  RRT step size
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
RRTOptions<dim>::RRTOptions(const float stepSize, const std::shared_ptr<CollisionDetection<dim>> &collision,
                            const std::shared_ptr<TrajectoryPlanner<dim>> &planner,
                            const std::shared_ptr<Sampling<dim>> &sampling, const std::shared_ptr<DistanceMetric<dim>> &metric)
    : PlannerOptions<dim>(collision, planner, sampling, metric) {
    setStepSize(stepSize);
}

/*!
*  \brief      Sets the step size of the RRTPlanner
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void RRTOptions<dim>::setStepSize(const float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Step size was smaller than 0 and was set up to 1", this);
        m_stepSize = 1;
    } else {
        m_stepSize = stepSize;
    }
}

/*!
*  \brief      Returns the step size of the RRTPlanner
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
float RRTOptions<dim>::getStepSize() const {
    return m_stepSize;
}

} /* namespace ippp */

#endif    // RRTOPTIONS_HPP
