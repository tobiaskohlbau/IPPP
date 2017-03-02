//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include <core/module/Identifier.h>
#include <core/module/TrajectoryPlanner.hpp>
#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <core/module/sampling/Sampling.hpp>
#include <core/utility/heuristic/Heuristic.hpp>

namespace rmpl {

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
                   const std::shared_ptr<Heuristic<dim>> &heuristic, const unsigned int sortingCountGraph);

    void setTrajectoryPlanner(const std::shared_ptr<TrajectoryPlanner<dim>> &planner);
    std::shared_ptr<TrajectoryPlanner<dim>> getTrajectoryPlanner() const;

    void setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision);
    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection() const;

    void setSampling(const std::shared_ptr<Sampling<dim>> &sampling);
    std::shared_ptr<Sampling<dim>> getSampling() const;

    void setHeuristic(const std::shared_ptr<Heuristic<dim>> &heuristic);
    std::shared_ptr<Heuristic<dim>> getHeuristic() const;

    void setSortCountGraph(const unsigned int sortAmountGraph);
    unsigned int getSortCountGraph() const;

  protected:
    float m_trajectoryStepSize;
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_planner = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<Heuristic<dim>> m_heuristic = nullptr;
    unsigned int m_sortingCountGraph = 0;
};

/*!
*  \brief      Standard constructor of the class PlannerOptions
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \param[in]  samplingMethod
*  \param[in]  Heuristic
*  \param[in]  sortingCountGraph
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
PlannerOptions<dim>::PlannerOptions(const std::shared_ptr<CollisionDetection<dim>> &collision,
                                    const std::shared_ptr<TrajectoryPlanner<dim>> &planner,
                                    const std::shared_ptr<Sampling<dim>> &sampling,
                                    const std::shared_ptr<Heuristic<dim>> &heuristic, const unsigned int sortingCountGraph)
    : Identifier("PlannerOptions") {
    m_collision = collision;
    m_planner = planner;
    m_heuristic = heuristic;
    m_sampling = sampling;
    m_sortingCountGraph = sortingCountGraph;
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

template <unsigned int dim>
void PlannerOptions<dim>::setCollisionDetection(const std::shared_ptr<CollisionDetection<dim>> &collision) {
    m_collision = collision;
}

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
*  \brief      Sets the EdgeHeuristic
*  \param[in]  heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
void PlannerOptions<dim>::setHeuristic(const std::shared_ptr<Heuristic<dim>> &heuristic) {
    m_heuristic = heuristic;
}

/*!
*  \brief      Returns the EdgeHeuristic
*  \param[out] heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
std::shared_ptr<Heuristic<dim>> PlannerOptions<dim>::getHeuristic() const {
    return m_heuristic;
}

/*!
* \brief      Sets count to sort Graph automically
* \param[in]  autoSort
* \author     Sascha Kaden
* \date       2017-01-05
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSortCountGraph(const unsigned int sortingCountGraph) {
    m_sortingCountGraph = sortingCountGraph;
}

/*!
*  \brief      Returns count to sort Graph automically
*  \param[out] autoSort
*  \author     Sascha Kaden
*  \date       2017-01-05
*/
template <unsigned int dim>
unsigned int PlannerOptions<dim>::getSortCountGraph() const {
    return m_sortingCountGraph;
}

} /* namespace rmpl */

#endif    // PLANNEROPTIONS_HPP
