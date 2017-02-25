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

#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <core/module/Identifier.h>
#include <core/module/Sampling.hpp>
#include <core/module/TrajectoryPlanner.hpp>
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
    PlannerOptions(float trajectoryStepSize, std::shared_ptr<CollisionDetection<dim>> collision, SamplerMethod samplerMethod, SamplingStrategy strategy,
                   std::shared_ptr<Heuristic<dim>> heuristic, unsigned int sortingCountGraph);

    void setTrajectoryStepSize(float stepSize);
    float getTrajectoryStepSize() const;

    void setCollisionDetection(std::shared_ptr<CollisionDetection<dim>> collision);
    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection() const;

    void setSamplerMethod(SamplerMethod method);
    SamplerMethod getSamplerMethod() const;
    void setSamplingStrategy(SamplingStrategy strategy);
    SamplingStrategy getSamplingStrategy() const;

    void setHeuristic(std::shared_ptr<Heuristic<dim>> heuristic);
    std::shared_ptr<Heuristic<dim>> getHeuristic() const;

    void setSortCountGraph(unsigned int sortAmountGraph);
    unsigned int getSortCountGraph() const;

  protected:
    float m_trajectoryStepSize;
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    SamplerMethod m_samplerMethod = SamplerMethod::randomly;
    SamplingStrategy m_samplingStrategy = SamplingStrategy::normal;
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
PlannerOptions<dim>::PlannerOptions(float trajectoryStepSize, std::shared_ptr<CollisionDetection<dim>> collision, SamplerMethod method, SamplingStrategy strategy,
                                    std::shared_ptr<Heuristic<dim>> heuristic, unsigned int sortingCountGraph) : Identifier("PlannerOptions") {
    setTrajectoryStepSize(trajectoryStepSize);
    m_collision = collision;
    setHeuristic(heuristic);
    m_samplingStrategy = strategy;
    m_samplerMethod = method;
    m_sortingCountGraph = sortingCountGraph;
}

/*!
*  \brief      Sets the trajectory step size
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setTrajectoryStepSize(float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set up to 1", this);
        m_trajectoryStepSize = 1;
    } else {
        m_trajectoryStepSize = stepSize;
    }
}

/*!
*  \brief      Returns the trajectory step size
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
float PlannerOptions<dim>::getTrajectoryStepSize() const {
    return m_trajectoryStepSize;
}

template <unsigned int dim>
void PlannerOptions<dim>::setCollisionDetection(std::shared_ptr<CollisionDetection<dim>> collision) {
    m_collision = collision;
}

template <unsigned int dim>
std::shared_ptr<CollisionDetection<dim>> PlannerOptions<dim>::getCollisionDetection() const {
    return m_collision;
}

/*!
*  \brief      Sets the sampling method
*  \param[in]  method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSamplerMethod(SamplerMethod method) {
    m_samplerMethod = method;
}

/*!
*  \brief      Returns the sampling method
*  \param[out] method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
template <unsigned int dim>
SamplerMethod PlannerOptions<dim>::getSamplerMethod() const {
    return m_samplerMethod;
}

/*!
*  \brief      Sets the sampling strategy
*  \param[in]  strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSamplingStrategy(SamplingStrategy strategy) {
    m_samplingStrategy = strategy;
}

/*!
*  \brief      Returns the strategy method
*  \param[out] strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
template <unsigned int dim>
SamplingStrategy PlannerOptions<dim>::getSamplingStrategy() const {
    return m_samplingStrategy;
}

/*!
*  \brief      Sets the EdgeHeuristic
*  \param[in]  heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
template <unsigned int dim>
void PlannerOptions<dim>::setHeuristic(std::shared_ptr<Heuristic<dim>> heuristic) {
    if (heuristic)
        m_heuristic = heuristic;
    else
        Logging::error("Empty Heuristic passed", this);
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
*  \brief      Sets count to sort Graph automically
*  \param[in]  autoSort
*  \author     Sascha Kaden
*  \date       2017-01-05
*/
template <unsigned int dim>
void PlannerOptions<dim>::setSortCountGraph(unsigned int sortingCountGraph) {
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
