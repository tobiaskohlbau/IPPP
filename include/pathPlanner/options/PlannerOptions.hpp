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

#ifndef PLANNEROPTIONS_HPP
#define PLANNEROPTIONS_HPP

#include <memory>

#include <core/Identifier.h>
#include <core/collisionDetection/CollisionDetection.hpp>
#include <core/distanceMetrics/DistanceMetric.hpp>
#include <core/sampling/Sampling.hpp>
#include <core/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {

/*!
* \brief   Class PlannerOptions determines all base options for the path planner
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class PlannerOptions : public Identifier {
  public:
    PlannerOptions(const std::shared_ptr<CollisionDetection<dim>> &collision,
                   const std::shared_ptr<TrajectoryPlanner<dim>> &planner, const std::shared_ptr<Sampling<dim>> &sampling,
                   const std::shared_ptr<DistanceMetric<dim>> &metric);

    void setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &planner);
    std::shared_ptr<TrajectoryPlanner<dim>> getTrajectoryPlanner() const;

    void setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision);
    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection() const;

    void setSampling(const std::shared_ptr<Sampling<dim>> &sampling);
    std::shared_ptr<Sampling<dim>> getSampling() const;

    void setDistanceMetric(const std::shared_ptr<DistanceMetric<dim>> &metric);
    std::shared_ptr<DistanceMetric<dim>> getDistanceMetric() const;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_planner = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
};

/*!
*  \brief      Standard constructor of the class PlannerOptions
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \param[in]  samplingMethod
*  \param[in]  DistanceMetric
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
PlannerOptions<dim>::PlannerOptions(const std::shared_ptr<CollisionDetection<dim>> &collision,
                                    const std::shared_ptr<TrajectoryPlanner<dim>> &planner,
                                    const std::shared_ptr<Sampling<dim>> &sampling,
                                    const std::shared_ptr<DistanceMetric<dim>> &metric)
    : Identifier("PlannerOptions") {
    m_collision = collision;
    m_planner = planner;
    m_metric = metric;
    m_sampling = sampling;
}

/*!
*  \brief      Sets the trajectory step size
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &planner) {
    m_planner = planner;
}

/*!
*  \brief      Returns the trajectory step size
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
std::shared_ptr<TrajectoryPlanner<dim>> PlannerOptions<dim>::getTrajectoryPlanner() const {
    return m_planner;
}

/*!
*  \brief      Set the collision detection
*  \param[in]  collision detection
*  \author     Sascha Kaden
*  \date       2016-01-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision) {
    m_collision = collision;
}

/*!
*  \brief      Returns the collision detection
*  \param[out] collision detection
*  \author     Sascha Kaden
*  \date       2016-01-29
*/
template <unsigned int dim>
std::shared_ptr<CollisionDetection<dim>> PlannerOptions<dim>::getCollisionDetection() const {
    return m_collision;
}

/*!
*  \brief      Sets the Sampling
*  \param[in]  strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSampling(const std::shared_ptr<Sampling<dim>> &sampling) {
    m_sampling = sampling;
}

/*!
*  \brief      Returns the Sampling
*  \param[out] sampling
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
std::shared_ptr<Sampling<dim>> PlannerOptions<dim>::getSampling() const {
    return m_sampling;
}

/*!
*  \brief      Sets the DistanceMetric
*  \param[in]  metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
void PlannerOptions<dim>::setDistanceMetric(const std::shared_ptr<DistanceMetric<dim>> &metric) {
    m_metric = metric;
}

/*!
*  \brief      Returns the DistanceMetric
*  \param[out] metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
std::shared_ptr<DistanceMetric<dim>> PlannerOptions<dim>::getDistanceMetric() const {
    return m_metric;
}

} /* namespace ippp */

#endif    // PLANNEROPTIONS_HPP
