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

#ifndef STRAIGHTSAMPLING_HPP
#define STRAIGHTSAMPLING_HPP

#include <ippp/modules/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class StraightSampling return just the sample of the passed Sampler
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class StraightSampling : public Sampling<dim> {
  public:
    StraightSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<Sampler<dim>> &sampler,
                     const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts = 10,
                     const std::string &name = "StraightSampling");

    virtual Vector<dim> getSample();

  protected:
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
*  \param[in]  attempts
*  \date       2016-12-20
*/
template <unsigned int dim>
StraightSampling<dim>::StraightSampling(const std::shared_ptr<Environment> &environment,
                                        const std::shared_ptr<Sampler<dim>> &sampler,
                                        const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts,
                                        const std::string &name)
    : Sampling<dim>(name, environment, nullptr, sampler, validityChecker, attempts) {
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> StraightSampling<dim>::getSample() {
    Vector<dim> sample;
    for (size_t i = 0; i < m_attempts; ++i) {
        sample = m_sampler->getSample();
        if (m_validityChecker->check(sample))
            return sample;
    }
    return util::NaNVector<dim>();
}

} /* namespace ippp */

#endif /* STRAIGHTSAMPLING_HPP */
