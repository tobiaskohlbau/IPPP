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

#ifndef GAUSSIANDISTSAMPLING_HPP
#define GAUSSIANDISTSAMPLING_HPP

#include <core/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Gaussian distance sampling creates samples in the near of the passed config.
* \author  Sascha Kaden
* \date    2017-06-07
*/
template <unsigned int dim>
class GaussianDistSampling : public Sampling<dim> {
  public:
    GaussianDistSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
                     const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
                     const unsigned int attempts = 10, const double maxDist = 15);

    Vector<dim> getSample() override;
    Vector<dim> getSample(const Vector<dim> &prevSample) override;

  private:
    double m_maxDist;

    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_collision;
    using Sampling<dim>::m_sampler;
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
                                        const std::shared_ptr<CollisionDetection<dim>> &collision,
                                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                        const std::shared_ptr<Sampler<dim>> &sampler, const unsigned int attempts,
                                        const double maxDist)
    : Sampling<dim>("GaussianDistSampling", environment, collision, trajectory, sampler, attempts), m_maxDist(maxDist) {
}

/*!
*  \brief      Return just sample from the Sampler
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> GaussianDistSampling<dim>::getSample() {
    return m_sampler->getSample();
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
    for (unsigned int count = 0; count < m_attempts; ++count) {
        ray = m_sampler->getRandomRay();
        ray *= m_maxDist * m_sampler->getRandomNumber();
        sample = prevSample + ray;
        if (!m_collision->controlVec(sample))
            return sample;
    }
    return util::NaNVector<dim>();
}

} /* namespace ippp */

#endif /* GAUSSIANDISTSAMPLING_HPP */
