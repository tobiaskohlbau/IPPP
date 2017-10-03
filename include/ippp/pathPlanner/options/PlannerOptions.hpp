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

#include <ippp/core/Identifier.h>
#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/core/distanceMetrics/DistanceMetric.hpp>
#include <ippp/core/evaluator/Evaluator.hpp>
#include <ippp/core/pathModifier/PathModifier.hpp>
#include <ippp/core/sampling/Sampling.hpp>
#include <ippp/core/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {

/*!
* \brief   Class PlannerOptions holds all module instances for the path planner.
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class PlannerOptions : public Identifier {
  public:
    PlannerOptions(const std::shared_ptr<CollisionDetection<dim>> &collision, const std::shared_ptr<DistanceMetric<dim>> &metric,
                   const std::shared_ptr<Evaluator<dim>> &evalutor, const std::shared_ptr<PathModifier<dim>> &pathModifier,
                   const std::shared_ptr<Sampling<dim>> &sampling, const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    void setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision);
    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection() const;

    void setDistanceMetric(const std::shared_ptr<DistanceMetric<dim>> &metric);
    std::shared_ptr<DistanceMetric<dim>> getDistanceMetric() const;

    void setEvaluator(const std::shared_ptr<Evaluator<dim>> &evaluator);
    std::shared_ptr<Evaluator<dim>> getEvaluator() const;

    void setPathModifier(const std::shared_ptr<PathModifier<dim>> &pathModifier);
    std::shared_ptr<PathModifier<dim>> getPathModifier() const;

    void setSampling(const std::shared_ptr<Sampling<dim>> &sampling);
    std::shared_ptr<Sampling<dim>> getSampling() const;

    void setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &planner);
    std::shared_ptr<TrajectoryPlanner<dim>> getTrajectoryPlanner() const;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Evaluator<dim>> m_evaluator = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Standard constructor of the class PlannerOptions, it holds all modules of the path planner.
*  \param[in]  CollisionDetection
*  \param[in]  DistanceMetric
*  \param[in]  Evaluator
*  \param[in]  PathModifier
*  \param[in]  Sampling
*  \param[in]  TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2017-05-26
*/
template <unsigned int dim>
PlannerOptions<dim>::PlannerOptions(const std::shared_ptr<CollisionDetection<dim>> &collision,
                                    const std::shared_ptr<DistanceMetric<dim>> &metric,
                                    const std::shared_ptr<Evaluator<dim>> &evaluator,
                                    const std::shared_ptr<PathModifier<dim>> &pathModifier,
                                    const std::shared_ptr<Sampling<dim>> &sampling,
                                    const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : Identifier("PlannerOptions"),
      m_collision(collision),
      m_metric(metric),
      m_evaluator(evaluator),
      m_pathModifier(pathModifier),
      m_sampling(sampling),
      m_trajectory(trajectory) {
}

/*!
*  \brief      Set the CollisionDetection instance.
*  \param[in]  collision detection
*  \author     Sascha Kaden
*  \date       2016-01-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision) {
    m_collision = collision;
}

/*!
*  \brief      Returns the CollisionDetection instance.
*  \param[out] collision detection
*  \author     Sascha Kaden
*  \date       2016-01-29
*/
template <unsigned int dim>
std::shared_ptr<CollisionDetection<dim>> PlannerOptions<dim>::getCollisionDetection() const {
    return m_collision;
}

/*!
*  \brief      Sets the DistanceMetric instance.
*  \param[in]  metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
void PlannerOptions<dim>::setDistanceMetric(const std::shared_ptr<DistanceMetric<dim>> &metric) {
    m_metric = metric;
}

/*!
*  \brief      Returns the DistanceMetric instance.
*  \param[out] metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
std::shared_ptr<DistanceMetric<dim>> PlannerOptions<dim>::getDistanceMetric() const {
    return m_metric;
}

/*!
*  \brief      Sets the Evaluator instance.
*  \param[in]  evaluator
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
template <unsigned int dim>
void PlannerOptions<dim>::setEvaluator(const std::shared_ptr<Evaluator<dim>> &evaluator) {
    m_evaluator = evaluator;
}

/*!
*  \brief      Returns the Evaluator instance.
*  \param[out] evaluator
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
template <unsigned int dim>
std::shared_ptr<Evaluator<dim>> PlannerOptions<dim>::getEvaluator() const {
    return m_evaluator;
}

/*!
*  \brief      Sets the PathModifier instance.
*  \param[in]  PathModifier
*  \author     Sascha Kaden
*  \date       2017-05-26
*/
template <unsigned int dim>
void PlannerOptions<dim>::setPathModifier(const std::shared_ptr<PathModifier<dim>> &pathModifier) {
    m_pathModifier = pathModifier;
}

/*!
*  \brief      Returns the PathModifier instance.
*  \param[out] PathModifier
*  \author     Sascha Kaden
*  \date       2017-05-26
*/
template <unsigned int dim>
std::shared_ptr<PathModifier<dim>> PlannerOptions<dim>::getPathModifier() const {
    return m_pathModifier;
}

/*!
*  \brief      Sets the Sampling instance.
*  \param[in]  Sampling
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSampling(const std::shared_ptr<Sampling<dim>> &sampling) {
    m_sampling = sampling;
}

/*!
*  \brief      Returns the Sampling instance.
*  \param[out] Sampling
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
std::shared_ptr<Sampling<dim>> PlannerOptions<dim>::getSampling() const {
    return m_sampling;
}

/*!
*  \brief      Sets the TrajectoryPlanner instance.
*  \param[in]  TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory) {
    m_trajectory = trajectory;
}

/*!
*  \brief      Returns the TrajectoryPlanner instance.
*  \param[out] TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
std::shared_ptr<TrajectoryPlanner<dim>> PlannerOptions<dim>::getTrajectoryPlanner() const {
    return m_trajectory;
}

} /* namespace ippp */

#endif    // PLANNEROPTIONS_HPP
