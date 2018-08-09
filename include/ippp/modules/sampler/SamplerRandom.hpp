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

#ifndef SAMPLERRANDOM_HPP
#define SAMPLERRANDOM_HPP

#include <ippp/modules/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   RandomSampler generates random samples within the boundaries of the robot(s).
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerRandom : public Sampler<dim> {
  public:
    SamplerRandom(const std::shared_ptr<Environment> &environment, const std::string &seed = "");
    SamplerRandom(const std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed = "");
    virtual Vector<dim> getSample();

  protected:
    using Sampler<dim>::m_robotBoundary;
    using Sampler<dim>::m_generator;
};

/*!
*  \brief      Constructor of the RandomSampler class
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerRandom<dim>::SamplerRandom(const std::shared_ptr<Environment> &environment, const std::string &seed)
    : Sampler<dim>("RandomSampler", environment, seed) {
}

/*!
*  \brief      Constructor of the RandomSampler class
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerRandom<dim>::SamplerRandom(const std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed)
    : Sampler<dim>("RandomSampler", boundary, seed) {
}

/*!
*  \brief      Return random sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> SamplerRandom<dim>::getSample() {
    Vector<dim> config;
    for (unsigned int i = 0; i < dim; ++i)
        config[i] =
            m_robotBoundary.first[i] + (double)(m_generator() % (int)(m_robotBoundary.second[i] - m_robotBoundary.first[i]));
    return config;
}

} /* namespace ippp */

#endif /* SAMPLERRANDOM_HPP */
