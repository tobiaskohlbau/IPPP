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

#include <ippp/modules/sampling/Sampling.hpp>

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
                         const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                         const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts,
                         const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    Vector<dim> getSample() override;

  private:
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;

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
SamplingNearObstacle<dim>::SamplingNearObstacle(const std::shared_ptr<Environment> &environment,
                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                                                const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts,
                                                const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : Sampling<dim>("SamplingNearObstacle", environment, validityChecker, sampler, attempts), m_trajectory(trajectory) {
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
    if (m_validityChecker->check(sample1)) {
        return sample1;
    } else {
        Vector<dim> sample2;
        for (size_t count = 0; count < m_attempts; ++count) {
            sample2 = m_sampler->getSample();
            if (!m_validityChecker->check(sample2)) {
                std::vector<Vector<dim>> path = m_trajectory->calcTrajCont(sample1, sample2);
                for (auto &point : path)
                    if (m_validityChecker->check(point))
                        return point;
                return sample2;
            }
        }
    }
    return util::NaNVector<dim>();
}

} /* namespace ippp */

#endif /* SAMPLINGNEAROBSTACLE_HPP */
