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

#ifndef RGDSAMPLING_HPP
#define RGDSAMPLING_HPP

#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/modules/sampler/SamplerNormalDist.hpp>
#include <ippp/modules/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class SamplingNearObstacle creates samples with the passed Sampler and if they in collision valid samples in
* neighborhood will found.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class RGDSampling : public Sampling<dim> {
  public:
    RGDSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts, const std::shared_ptr<Graph<dim>> &graph);

    Vector<dim> getSample() override;
    Vector<dim> getSample(const Vector<dim> &prevSample) override;

  private:
    double m_epsilon;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;

    using Sampling<dim>::m_attempts;
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
*  \date       2016-12-20
*/
template <unsigned int dim>
RGDSampling<dim>::RGDSampling(const std::shared_ptr<Environment> &environment,
                              const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                              const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts,
                              const std::shared_ptr<Graph<dim>> &graph)
    : Sampling<dim>("RGDSampling", environment, validityChecker, sampler, attempts),
      m_graph(graph),
      m_epsilon(validityChecker->getEpsilon()) {
}

/*!
*  \brief      Sample in the neighborhood of obstacles
*  \details    If Sample is in collision, second random collision valid sample will be computed and by binary search the
*              nearest collision valid sample to the first sample, will be taken.
*  \author     Sascha Kaden
*  \param[out] sample Vector
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> RGDSampling<dim>::getSample() {
    return getSample(m_graph->getNode(static_cast<size_t>(this->getRandomNumber() * m_graph->numNodes()))->getValues());
}

template <unsigned int dim>
Vector<dim> RGDSampling<dim>::getSample(const Vector<dim> &prevSample) {
    double prevError = m_validityChecker->calc(prevSample);

    if (prevError <= 0)
        return prevSample;

    SamplerNormalDist<dim> normalSampler(this->m_environment);
    normalSampler.setOrigin(prevSample);
    Vector<dim> sample;
    double error;

    for (size_t i = 0; i < m_attempts; ++i) {
        sample = normalSampler.getSample();
        if (util::empty<dim>(sample))
            continue;

        error = m_validityChecker->calc(sample);
        if (error <= 0)
            return sample;

        if (error < prevError) {
            prevError = error;
            normalSampler.setOrigin(sample);
        }
        std::cout << "attempt: " << i << std::endl;
    }
    return util::NaNVector<dim>();
}

} /* namespace ippp */

#endif /* RGDSAMPLING_HPP */
