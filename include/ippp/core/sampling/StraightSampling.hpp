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

#ifndef STRAIGHTSAMPLING_HPP
#define STRAIGHTSAMPLING_HPP

#include <ippp/core/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class Sampling creates sample Vectors with the passed strategy, for creating single Vectors the Sampler is used.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class StraightSampling : public Sampling<dim> {
  public:
    StraightSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
             const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler);

    virtual Vector<dim> getSample();

  protected:
    using Sampling<dim>::m_sampler;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \param[in]  Sampler
*  \param[in]  Sampler
*  \date       2016-12-20
*/
template <unsigned int dim>
StraightSampling<dim>::StraightSampling(const std::shared_ptr<Environment> &environment,
                        const std::shared_ptr<CollisionDetection<dim>> &collision,
                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler)
    : Sampling<dim>("StraightSampling", environment, collision, trajectory, sampler, 10) {
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> StraightSampling<dim>::getSample() {
    return m_sampler->getSample();
}

} /* namespace ippp */

#endif /* STRAIGHTSAMPLING_HPP */
