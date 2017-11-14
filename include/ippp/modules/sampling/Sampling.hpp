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

#ifndef SAMPLING_HPP
#define SAMPLING_HPP

#include <math.h>

#include <Eigen/Core>

#include <ippp/Identifier.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/modules/sampler/Sampler.hpp>
#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>
#include <ippp/environment/Environment.h>

namespace ippp {

/*!
* \brief   Class Sampling creates sample Vectors with the passed strategy, for creating single Vectors the Sampler is used.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class Sampling : public Identifier {
  public:
    Sampling(const std::string &name, const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
             const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
             const size_t attempts = 10);

    virtual Vector<dim> getSample() = 0;
    virtual Vector<dim> getSample(const Vector<dim> &prevSample);
    virtual std::vector<Vector<dim>> getSamples(const unsigned int amount);

    std::shared_ptr<Sampler<dim>> getSampler() const;
    double getRandomNumber() const;
    void setOrigin(const Vector<dim> &origin);

  protected:
    const size_t m_attempts;

    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Sampler<dim>> m_sampler = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \param[in]  Sampler
*  \param[in]  attempts for one sampling
*  \date       2016-12-20
*/
template <unsigned int dim>
Sampling<dim>::Sampling(const std::string &name, const std::shared_ptr<Environment> &environment,
                        const std::shared_ptr<CollisionDetection<dim>> &collision,
                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory, const std::shared_ptr<Sampler<dim>> &sampler,
                        const size_t attempts)
    : Identifier(name), m_collision(collision), m_trajectory(trajectory), m_sampler(sampler), m_attempts(attempts) {
    Logging::debug("Initialize", this);
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[in]  previous sample
*  \param[out] sample
*  \date       2017-06-07
*/
template <unsigned int dim>
Vector<dim> Sampling<dim>::getSample(const Vector<dim> &prevSample) {
    return getSample();
}

/*!
*  \brief      Return list of samples
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[out] list of samples
*  \date       2017-06-07
*/
template <unsigned int dim>
std::vector<Vector<dim>> Sampling<dim>::getSamples(const unsigned int amount) {
    std::vector<Vector<dim>> samples;
    samples.reserve(amount);
    for (unsigned int i = 0; i < amount; ++i)
        samples.push_back(getSample());

    return samples;
}

/*!
*  \brief      Return the Sampler of the Sampling strategy.
*  \author     Sascha Kaden
*  \param[out] Sampler instance
*  \date       2017-11-14
*/
template <unsigned int dim>
std::shared_ptr<Sampler<dim>> Sampling<dim>::getSampler() const {
    return m_sampler;
}

/*!
*  \brief      Return random number between 0 and 1
*  \author     Sascha Kaden
*  \param[out] random number
*  \date       2017-04-03
*/
template <unsigned int dim>
double Sampling<dim>::getRandomNumber() const {
    return m_sampler->getRandomNumber();
}

/*!
*  \brief      Set the origin of the Sampler
*  \author     Sascha Kaden
*  \param[in]  origin
*  \date       2016-12-20
*/
template <unsigned int dim>
void Sampling<dim>::setOrigin(const Vector<dim> &origin) {
    m_sampler->setOrigin(origin);
}

} /* namespace ippp */

#endif /* SAMPLING_HPP */
