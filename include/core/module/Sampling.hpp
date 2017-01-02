//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <math.h>

#include <Eigen/Core>

#include <core/module/CollisionDetection.hpp>
#include <core/module/ModuleBase.h>
#include <core/module/Sampler.hpp>
#include <core/module/TrajectoryPlanner.hpp>
#include <robot/RobotBase.hpp>

namespace rmpl {

enum SamplingStrategy { normal, nearObstacles };

/*!
* \brief   Class Sampling creates sample Vectors with the passed strategy, for creating single Vectors the Sampler is used.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class Sampling : public ModuleBase {
  public:
    Sampling(const std::shared_ptr<RobotBase<dim>> &robot, const std::shared_ptr<CollisionDetection<dim>> &collision,
             const std::shared_ptr<TrajectoryPlanner<dim>> &planner, SamplerMethod method = SamplerMethod::randomly,
             SamplingStrategy strategy = SamplingStrategy::normal);

    Vector<dim> getSample();
    bool setMeanOfDistribution(const Vector<dim> &mean);

  private:
    Vector<dim> sampleNearObstacle();

    SamplingStrategy m_strategy;

    std::shared_ptr<CollisionDetection<dim>> m_collision;
    std::shared_ptr<Sampler<dim>> m_sampler;
    std::shared_ptr<TrajectoryPlanner<dim>> m_planner;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplerMethod
*  \param[in]  SamplingStrategy
*  \date       2016-12-20
*/
template <unsigned int dim>
Sampling<dim>::Sampling(const std::shared_ptr<RobotBase<dim>> &robot, const std::shared_ptr<CollisionDetection<dim>> &collision,
                        const std::shared_ptr<TrajectoryPlanner<dim>> &planner, SamplerMethod method, SamplingStrategy strategy)
    : ModuleBase("Sampling") {
    m_strategy = strategy;

    m_collision = collision;
    m_planner = planner;
    m_sampler = std::shared_ptr<Sampler<dim>>(new Sampler<dim>(robot, method));
}

/*!
*  \brief      Return sample by the specified SamplingStrategy
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> Sampling<dim>::getSample() {
    if (m_strategy == SamplingStrategy::nearObstacles)
        return sampleNearObstacle();
    else
        return m_sampler->getSample();
}

/*!
*  \brief      Sample in the neighborhood of obstacles
*  \details    If Sample is in collision, second random collision free sample will be computed and by binary search the
*              nearest collision free sample to the first sample, will be taken.
*  \author     Sascha Kaden
*  \param[out] sample Vector
*  \date       2016-12-20
*/
template <unsigned int dim>
    Vector<dim> Sampling<dim>::sampleNearObstacle() {
        Vector<dim> sample1 = m_sampler->getSample();
    if (!m_collision->controlVec(sample1)) {
        return sample1;
    } else {
        Vector<dim> sample2;
        do {
            sample2 = m_sampler->getSample();
        } while (m_collision->controlVec(sample2));
        std::vector<Vector<dim>> path = m_planner->calcTrajectoryBin(sample2, sample1);
        sample1 = path[0];
        for (auto point : path) {
            if (!m_collision->controlVec(point))
                sample1 = point;
            else
                break;
        }
        return sample1;
    }
}

/*!
*  \brief      Set the mean of the standard distribution of the Sampler
*  \author     Sascha Kaden
*  \param[in]  mean of distribution
*  \param[out] binary result
*  \date       2016-12-20
*/
template <unsigned int dim>
bool Sampling<dim>::setMeanOfDistribution(const Vector<dim> &mean) {
    return m_sampler->setMeanOfDistribution(mean);
}

} /* namespace rmpl */

#endif /* SAMPLING_H_ */
