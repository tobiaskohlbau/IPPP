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

#ifndef MEDIALAXISSAMPLING_HPP
#define MEDIALAXISSAMPLING_HPP

#include <ippp/core/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   MedialAxisSampling creates approximated samples on the medial axis.
* \author  Sascha Kaden
* \date    2017-06-19
*/
template <unsigned int dim>
class MedialAxisSampling : public Sampling<dim> {
  public:
    MedialAxisSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
                       const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
                       const size_t attempts = 10, const size_t numDirs = 50);

    Vector<dim> getSample() override;

  private:
    size_t m_numberDirections;
    std::vector<Vector<dim>> m_directions;

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
*  \param[in]  number of directions
*  \date       2017-06-19
*/
template <unsigned int dim>
MedialAxisSampling<dim>::MedialAxisSampling(const std::shared_ptr<Environment> &environment,
                                            const std::shared_ptr<CollisionDetection<dim>> &collision,
                                            const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                            const std::shared_ptr<Sampler<dim>> &sampler, const size_t attempts,
                                            const size_t numberDirs)
    : Sampling<dim>("MedialAxisSampling", environment, collision, trajectory, sampler, attempts), m_numberDirections(numberDirs) {
    m_directions.reserve(m_numberDirections);
    for (unsigned int i = 0; i < numberDirs; ++i)
        m_directions.push_back(m_sampler->getRandomRay());
}

/*!
*  \brief      Return medial axis sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2017-06-19
*/
template <unsigned int dim>
Vector<dim> MedialAxisSampling<dim>::getSample() {
    auto sample = m_sampler->getSample();
    bool sampleCollision = m_collision->checkConfig(sample);

    std::vector<Vector<dim>> tempConfigs(m_numberDirections, sample);
    Vector<dim> first, direction;
    bool collision = sampleCollision;

    // calculate the nearest obstacle position with the computed directions
    while (collision == sampleCollision) {
        auto temp = tempConfigs.begin();
        for (auto dir = m_directions.begin(); dir < m_directions.end(); ++dir, ++temp) {
            *temp += *dir;
            if (m_collision->checkConfig(*temp) != sampleCollision) {
                // set the first collision vector and the direction and break the while loop
                first = *temp;
                direction = *dir;
                collision = !sampleCollision;
                break;
            }
        }
    }

    // compute the next collision in the opposite direction
    Vector<dim> second = sample;
    if (sampleCollision)
        second = first;
    collision = false;
    while (!collision) {
        if (!sampleCollision)
            second -= direction;
        else
            second += direction;

        collision = m_collision->checkConfig(second);
    }
    // return the middle point of these two collisions
    return second - ((second - first) / 2);
}

} /* namespace ippp */

#endif /* MEDIALAXISSAMPLING_HPP */
