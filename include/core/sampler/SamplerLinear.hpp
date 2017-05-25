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

#ifndef SAMPLERLINEAR_HPP
#define SAMPLERLINEAR_HPP

#include <core/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   SamplerLinear creates random samples with fixed orientation for 6 dimensional robots.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerLinear : public Sampler<dim> {
  public:
    SamplerLinear(const std::shared_ptr<Environment> &environment);
    Vector<dim> getSample() override;

  private:
};

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerLinear<dim>::SamplerLinear(const std::shared_ptr<Environment> &environment) : Sampler<dim>(environment, "LinearSampler") {
}

/*!
*  \brief      Return random sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> SamplerLinear<dim>::getSample() {
    Vector<dim> vec;
    if (dim == 6) {
        for (unsigned int i = 0; i < 3; ++i) {
            vec[i] = m_minBoundary[i] + (double)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
        }
        for (unsigned int i = 3; i < 6; ++i) {
            vec[i] = this->m_origin[i - 3];
        }
    } else {
        for (unsigned int i = 0; i < dim; ++i) {
            vec[i] = m_minBoundary[i] + (double)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
        }
    }
    return vec;
}

} /* namespace ippp */

#endif /* SAMPLERLINEAR_HPP */
