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

#ifndef MPOPTIONS_HPP
#define MPOPTIONS_HPP

#include <memory>

#include <ippp/Identifier.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/modules/distanceMetrics/DistanceMetric.hpp>
#include <ippp/modules/evaluator/Evaluator.hpp>
#include <ippp/modules/pathModifier/PathModifier.hpp>
#include <ippp/modules/sampling/Sampling.hpp>
#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {

/*!
* \brief   Class MPOptions holds all module instances from the MotionPlanner.
* \author  Sascha Kaden
* \date    2016-08-29
*/
template <unsigned int dim>
class MPOptions : public Identifier {
  public:
    MPOptions(const std::shared_ptr<ValidityChecker<dim>> &validityChecker, const std::shared_ptr<DistanceMetric<dim>> &metric,
              const std::shared_ptr<Evaluator<dim>> &evalutor, const std::shared_ptr<PathModifier<dim>> &pathModifier,
              const std::shared_ptr<Sampling<dim>> &sampling, const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    void setValidityChecker(const std::shared_ptr<ValidityChecker<dim>> &collision);
    std::shared_ptr<ValidityChecker<dim>> getValidityChecker() const;

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
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;
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
MPOptions<dim>::MPOptions(const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                          const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Evaluator<dim>> &evaluator,
                          const std::shared_ptr<PathModifier<dim>> &pathModifier, const std::shared_ptr<Sampling<dim>> &sampling,
                          const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : Identifier("PlannerOptions"),
      m_validityChecker(validityChecker),
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
void MPOptions<dim>::setValidityChecker(const std::shared_ptr<ValidityChecker<dim>> &validityChecker) {
    m_validityChecker = validityChecker;
}

/*!
*  \brief      Returns the CollisionDetection instance.
*  \param[out] collision detection
*  \author     Sascha Kaden
*  \date       2016-01-29
*/
template <unsigned int dim>
std::shared_ptr<ValidityChecker<dim>> MPOptions<dim>::getValidityChecker() const {
    return m_validityChecker;
}

/*!
*  \brief      Sets the DistanceMetric instance.
*  \param[in]  metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
void MPOptions<dim>::setDistanceMetric(const std::shared_ptr<DistanceMetric<dim>> &metric) {
    m_metric = metric;
}

/*!
*  \brief      Returns the DistanceMetric instance.
*  \param[out] metric
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
std::shared_ptr<DistanceMetric<dim>> MPOptions<dim>::getDistanceMetric() const {
    return m_metric;
}

/*!
*  \brief      Sets the Evaluator instance.
*  \param[in]  evaluator
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
template <unsigned int dim>
void MPOptions<dim>::setEvaluator(const std::shared_ptr<Evaluator<dim>> &evaluator) {
    m_evaluator = evaluator;
}

/*!
*  \brief      Returns the Evaluator instance.
*  \param[out] evaluator
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
template <unsigned int dim>
std::shared_ptr<Evaluator<dim>> MPOptions<dim>::getEvaluator() const {
    return m_evaluator;
}

/*!
*  \brief      Sets the PathModifier instance.
*  \param[in]  PathModifier
*  \author     Sascha Kaden
*  \date       2017-05-26
*/
template <unsigned int dim>
void MPOptions<dim>::setPathModifier(const std::shared_ptr<PathModifier<dim>> &pathModifier) {
    m_pathModifier = pathModifier;
}

/*!
*  \brief      Returns the PathModifier instance.
*  \param[out] PathModifier
*  \author     Sascha Kaden
*  \date       2017-05-26
*/
template <unsigned int dim>
std::shared_ptr<PathModifier<dim>> MPOptions<dim>::getPathModifier() const {
    return m_pathModifier;
}

/*!
*  \brief      Sets the Sampling instance.
*  \param[in]  Sampling
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
void MPOptions<dim>::setSampling(const std::shared_ptr<Sampling<dim>> &sampling) {
    m_sampling = sampling;
}

/*!
*  \brief      Returns the Sampling instance.
*  \param[out] Sampling
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
std::shared_ptr<Sampling<dim>> MPOptions<dim>::getSampling() const {
    return m_sampling;
}

/*!
*  \brief      Sets the TrajectoryPlanner instance.
*  \param[in]  TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void MPOptions<dim>::setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory) {
    m_trajectory = trajectory;
}

/*!
*  \brief      Returns the TrajectoryPlanner instance.
*  \param[out] TrajectoryPlanner
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
std::shared_ptr<TrajectoryPlanner<dim>> MPOptions<dim>::getTrajectoryPlanner() const {
    return m_trajectory;
}

} /* namespace ippp */

#endif    // MPOPTIONS_HPP
