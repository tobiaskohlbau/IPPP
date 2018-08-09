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

#ifndef SAMPLERUNIFORM_HPP
#define SAMPLERUNIFORM_HPP

#include <ippp/modules/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   Class SamplerUniform creates uniform samples, area is from the robot boundaries defined.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerUniform : public Sampler<dim> {
  public:
    SamplerUniform(const std::shared_ptr<Environment> &environment, const std::string &seed = "");
    SamplerUniform(const std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed = "");
    Vector<dim> getSample() override;

  private:
    std::vector<std::uniform_real_distribution<double>> m_distUniform;

    using Sampler<dim>::m_robotBoundary;
    using Sampler<dim>::m_generator;
};

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerUniform<dim>::SamplerUniform(const std::shared_ptr<Environment> &environment, const std::string &seed)
    : Sampler<dim>("SamplerUniform", environment, seed) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(m_robotBoundary.first[i], m_robotBoundary.second[i]);
        m_distUniform.push_back(dist);
    }
}

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerUniform<dim>::SamplerUniform(std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed)
    : Sampler<dim>("SamplerUniform", boundary, seed) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(m_robotBoundary.first[i], m_robotBoundary.second[i]);
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
    Vector<dim> config;
    for (unsigned int i = 0; i < dim; ++i)
        config[i] = m_distUniform[i](m_generator);

    return config;
}

} /* namespace ippp */

#endif /* SAMPLERUNIFORM_HPP */
