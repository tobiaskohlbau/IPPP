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

#ifndef SAMPLINGNEAROBSTACLE_HPP
#define SAMPLINGNEAROBSTACLE_HPP

#include <ippp/core/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class SamplingNearObstacle creates samples with the passed Sampler and if they in collision valid samples in
* neighborhood will found.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class SamplingNearObstacle : public Sampling<dim> {
  public:
    SamplingNearObstacle(const std::shared_ptr<Environment> &environment,
                         const std::shared_ptr<CollisionDetection<dim>> &collision,
                         const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
                         const unsigned int attempts = 10);

    Vector<dim> getSample() override;

  private:
    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_sampler;
    using Sampling<dim>::m_collision;
    using Sampling<dim>::m_trajectory;
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
SamplingNearObstacle<dim>::SamplingNearObstacle(const std::shared_ptr<Environment> &environment,
                                                const std::shared_ptr<CollisionDetection<dim>> &collision,
                                                const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                                const std::shared_ptr<Sampler<dim>> &sampler, const unsigned int attempts)
    : Sampling<dim>("SamplingNearObstacle", environment, collision, trajectory, sampler, attempts) {
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
Vector<dim> SamplingNearObstacle<dim>::getSample() {
    Vector<dim> sample1 = m_sampler->getSample();
    if (!m_collision->checkConfig(sample1)) {
        return sample1;
    } else {
        Vector<dim> sample2;
        do {
            sample2 = m_sampler->getSample();
        } while (m_collision->checkConfig(sample2));
        std::vector<Vector<dim>> path = m_trajectory->calcTrajectoryBin(sample2, sample1);
        sample1 = path[0];
        for (auto point : path) {
            if (!m_collision->checkConfig(point))
                sample1 = point;
            else
                break;
        }
        return sample1;
    }
}

} /* namespace ippp */

#endif /* SAMPLINGNEAROBSTACLE_HPP */
