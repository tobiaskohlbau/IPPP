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

#ifndef BRIDGESAMPLING_HPP
#define BRIDGESAMPLING_HPP

#include <core/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class Sampling creates sample Vectors with the passed strategy, for creating single Vectors the Sampler is used.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class BridgeSampling : public Sampling<dim> {
  public:
    BridgeSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
                   const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
                   const unsigned int attempts = 10, const double distance = 15);

    virtual Vector<dim> getSample();

  private:
    double m_distance;

    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_collision;
    using Sampling<dim>::m_sampler;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \param[in]  Sampler
*  \date       2016-12-20
*/
template <unsigned int dim>
BridgeSampling<dim>::BridgeSampling(const std::shared_ptr<Environment> &environment,
                                    const std::shared_ptr<CollisionDetection<dim>> &collision,
                                    const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                    const std::shared_ptr<Sampler<dim>> &sampler, const unsigned int attempts,
                                    const double distance)
    : Sampling<dim>("BridgeSampling", environment, collision, trajectory, sampler, attempts), m_distance(distance) {
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> BridgeSampling<dim>::getSample() {
    unsigned int count = 0;
    Vector<dim> sample1, sample2, ray;
    do {
        sample1 = m_sampler->getSample();
    } while (!m_collision->controlVec(sample1));

    do {
        if (count > m_attempts)
            return util::NaNVector<dim>();
        else
            ++count;

        ray = m_sampler->getRandomRay();
        ray *= m_distance * m_sampler->getRandomNumber();
        sample2 = sample1 + ray;
    } while (!m_collision->controlVec(sample2) && m_collision->controlVec(sample1 + (ray / 2)));

    return sample1 + (ray / 2);
}

} /* namespace ippp */

#endif /* BRIDGESAMPLING_HPP */
