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

#ifndef SAMPLERUNIFORM_HPP
#define SAMPLERUNIFORM_HPP

#include <core/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   Class SamplerUniform creates uniform samples, area is from the robot boundaries defined.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerUniform : public Sampler<dim> {
  public:
    SamplerUniform(const std::shared_ptr<Environment> &environment);
    Vector<dim> getSample() override;

  private:
    std::vector<std::uniform_real_distribution<double>> m_distUniform;
};

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerUniform<dim>::SamplerUniform(const std::shared_ptr<Environment> &environment)
    : Sampler<dim>("SamplerUniform", environment) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(this->m_minBoundary[i], this->m_maxBoundary[i]);
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
Vector<dim> SamplerUniform<dim>::getSample() {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = m_distUniform[i](this->m_generator);
    }
    return vec;
}

} /* namespace ippp */

#endif /* SAMPLERUNIFORM_HPP */
