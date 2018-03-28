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

#ifndef SAMPLERUNIFORMBIASED_HPP
#define SAMPLERUNIFORMBIASED_HPP

#include <ippp/dataObj/Graph.hpp>
#include <ippp/modules/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   Class SamplerUniformBiased creates uniform samples, area is from the robot boundaries defined.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerUniformBiased : public Sampler<dim> {
  public:
    SamplerUniformBiased(const std::shared_ptr<Environment> &environment, const std::shared_ptr<Graph<dim>> &graph,
                         const std::string &seed = "");
    SamplerUniformBiased(const std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed = "");
    Vector<dim> getSample() override;

  private:
    std::vector<std::uniform_real_distribution<double>> m_distUniform;
    std::shared_ptr<Graph<dim>> m_graph;

    using Sampler<dim>::m_generator;
    using Sampler<dim>::m_optimalPathCost;
    using Sampler<dim>::m_robotBoundary;
};

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerUniformBiased<dim>::SamplerUniformBiased(const std::shared_ptr<Environment> &environment,
                                                const std::shared_ptr<Graph<dim>> &graph, const std::string &seed)
    : Sampler<dim>("SamplerUniformBiased", environment, seed), m_graph(graph) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(m_robotBoundary.first[i], m_robotBoundary.second[i]);
        m_distUniform.push_back(dist);
    }
}

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerUniformBiased<dim>::SamplerUniformBiased(std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed)
    : Sampler<dim>("SamplerUniformBiased", boundary, seed) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(m_robotBoundary.first[i], m_robotBoundary.second[i]);
        m_distUniform.push_back(dist);
    }
}

/*!
*  \brief      Return uniform sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> SamplerUniformBiased<dim>::getSample() {
    Vector<dim> config;
    double maxCost = m_graph->getMaxNodeCost();
    for (size_t i = 0; i < 10; ++i) {
        for (unsigned int i = 0; i < dim; ++i)
            config[i] = m_distUniform[i](m_generator);

        auto nearest = m_graph->getNearestNode(config);
        if (!nearest || this->getRandomNumber() > 1 - (nearest->getCost() - m_optimalPathCost) / (maxCost - m_optimalPathCost))
            return config;
    }
    return config;
}

} /* namespace ippp */

#endif /* SAMPLERUNIFORMBIASED_HPP */
