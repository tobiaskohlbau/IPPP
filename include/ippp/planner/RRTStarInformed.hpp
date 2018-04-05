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

#ifndef RRTSTARINFORMED_HPP
#define RRTSTARINFORMED_HPP

#include <ippp/modules/sampler/EllipsoidSampler.hpp>
#include <ippp/planner/RRTStar.hpp>

namespace ippp {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2018-03-2
*/
template <unsigned int dim>
class RRTStarInformed : public RRTStar<dim> {
  public:
    RRTStarInformed(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
                    const std::shared_ptr<Graph<dim>> &graph);

    bool optimize(size_t numNodes, size_t numThreads = 1) override;

  protected:
    using Planner<dim>::m_validityChecker;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_evaluator;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_plannerCollector;
    using TreePlanner<dim>::m_initNode;
    using TreePlanner<dim>::m_goalNode;
};

/*!
*  \brief      Standard constructor of the class StarRRTPlanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  options
*  \date       2017-02-19
*/
template <unsigned int dim>
RRTStarInformed<dim>::RRTStarInformed(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
                                      const std::shared_ptr<Graph<dim>> &graph)
    : RRTStar<dim>(environment, options, graph, "RRTStarInformed") {
}

/*!
*  \brief      Compute path from start Node<dim> to goal Node<dim> with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start configuartion
*  \param[in]  goal configuartion
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2017-06-20
*/
template <unsigned int dim>
bool RRTStarInformed<dim>::optimize(size_t numNodes, size_t numThreads) {
    m_plannerCollector->startOptimizationTimer();
    if (!m_pathPlanned) {
        Logging::warning("No optimization, because no plan is planned", this);
        return false;
    }
    Logging::debug("Optimization", this);

    auto oldSampler = this->m_sampling->getSampler();
    auto ellipsoidSampler = std::make_shared<EllipsoidSampler<dim>>(m_environment);
    this->m_sampling->setSampler(ellipsoidSampler);

    ellipsoidSampler->setParams(m_initNode->getValues(), m_goalNode->getValues(), m_goalNode->getCost());
    this->expand(numNodes, numThreads);

    this->m_sampling->setSampler(oldSampler);

    m_plannerCollector->stopOptimizationTimer();
    this->updateStats();
    return true;
}

} /* namespace ippp */

#endif /* RRTSTARINFORMED_HPP */
