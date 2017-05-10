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

#ifndef SEEDSAMPLER_HPP
#define SEEDSAMPLER_HPP

#include <core/module/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   SeedSampler creates random samples with the passed seed. If no seed is passed, standard seed will be used.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SeedSampler : public Sampler<dim> {
  public:
    SeedSampler(const std::shared_ptr<RobotBase<dim>> &robot, const std::string &seed = "akls23fd43253haosrel234lh234kj2g3h42g");
    virtual Vector<dim> getSample() override;

  protected:
    std::minstd_rand0 m_randomEngine;
};

/*!
*  \brief      Constructor of the SeedSampler
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2016-05-24
*/
template <unsigned int dim>
SeedSampler<dim>::SeedSampler(const std::shared_ptr<RobotBase<dim>> &robot, const std::string &seed) : Sampler<dim>(robot, "SeedSampler") {
    std::seed_seq seed_seq(seed.begin(), seed.end());
    m_randomEngine = std::minstd_rand0(seed_seq);
}

/*!
*  \brief      Return random seed sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> SeedSampler<dim>::getSample() {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = this->m_minBoundary[i] + (float)(m_randomEngine() % (int)(this->m_maxBoundary[i] - this->m_minBoundary[i]));
    }
    return vec;
}

} /* namespace ippp */

#endif /* SEEDSAMPLER_HPP */
