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

#ifndef GAUSSIANDISTSAMPLING_HPP
#define GAUSSIANDISTSAMPLING_HPP

#include <ippp/modules/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Gaussian distance sampling creates samples in the near of the passed config.
* \author  Sascha Kaden
* \date    2017-06-07
*/
template <unsigned int dim>
class GaussianDistSampling : public Sampling<dim> {
  public:
    GaussianDistSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<Graph<dim>> &graph,
                         const std::shared_ptr<Sampler<dim>> &sampler,
                         const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts, double maxDist = 15,
                         const std::string &name = "GaussianDistSampling");

    Vector<dim> getSample() override;
    Vector<dim> getSample(const Vector<dim> &prevSample) override;

  private:
    double m_maxDist;

    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_graph;
    using Sampling<dim>::m_sampler;
    using Sampling<dim>::m_validityChecker;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \param[in]  Sampler
*  \param[in]  attempts for one sampling
*  \param[in]  max distance to previous sample
*  \date        2017-06-07
*/
template <unsigned int dim>
GaussianDistSampling<dim>::GaussianDistSampling(const std::shared_ptr<Environment> &environment,
                                                const std::shared_ptr<Graph<dim>> &graph,
                                                const std::shared_ptr<Sampler<dim>> &sampler,
                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts,
                                                double maxDist, const std::string &name)
    : Sampling<dim>(name, environment, graph, sampler, validityChecker, attempts), m_maxDist(maxDist) {
}

/*!
*  \brief      Return just sample from the Sampler
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> GaussianDistSampling<dim>::getSample() {
    return getSample(m_graph->getNode(static_cast<size_t>(this->getRandomNumber() * m_graph->numNodes()))->getValues());
}

/*!
*  \brief      Return free sample in the near of the previous sample.
*  \details    The distance to the previous sample will be randomly from zero to max distance.
*  \author     Sascha Kaden
*  \param[in]  previous sample
*  \param[out] sample
*  \date       2017-06-07
*/
template <unsigned int dim>
Vector<dim> GaussianDistSampling<dim>::getSample(const Vector<dim> &prevSample) {
    Vector<dim> ray, sample;
    for (size_t count = 0; count < m_attempts; ++count) {
        ray = m_sampler->getRandomRay();
        ray *= m_maxDist * m_sampler->getRandomNumber();
        sample = prevSample + ray;
        if (m_validityChecker->checkRobotBound(sample) && m_validityChecker->check(sample))
            return sample;
    }
    return util::NaNVector<dim>();
}

} /* namespace ippp */

#endif /* GAUSSIANDISTSAMPLING_HPP */
