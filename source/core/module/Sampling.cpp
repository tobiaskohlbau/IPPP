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

#include <core/module/Sampling.h>

namespace rmpl {

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \param[in]  SamplingStrategy
*  \date       2016-12-20
*/
Sampling::Sampling(const std::shared_ptr<RobotBase> &robot, const std::shared_ptr<CollisionDetection> &collision,
                   const std::shared_ptr<TrajectoryPlanner> &planner, SamplingMethod method, SamplingStrategy strategy)
    : ModuleBase("Sampling") {
    m_strategy = strategy;

    m_collision = collision;
    m_robot = robot;
    m_planner = planner;
    m_sampler = std::shared_ptr<Sampler>(new Sampler(robot, method));
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
Vec<float> Sampling::getSample() {
    if (m_strategy == SamplingStrategy::nearObstacles)
        return sampleNearObstacle();
    else
        return m_sampler->getSample();
}

/*!
*  \brief      Sample in the neighboorhood of obstacles
*  \details    If Sample is in collision, second random collision free sample will be computed and by binary search the
*              nearest collision free sample to the first sample, will be taken.
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-12-20
*/
Vec<float> Sampling::sampleNearObstacle() {
    Vec<float> sample1 = m_sampler->getSample();
    if (!m_collision->controlVec(sample1)) {
        return sample1;
    } else {
        Vec<float> sample2;
        do {
            sample2 = m_sampler->getSample();
        } while (m_collision->controlVec(sample2));
        std::vector<Vec<float>> path = m_planner->computeTrajectory(sample2, sample1);
        sample1 = path[0];
        for (auto point : path) {
            if (!m_collision->controlVec(point))
                sample1 = point;
            else
                break;
        }
    }
}

/*!
*  \brief      Set the mean of the standard distribution of the Sampler
*  \author     Sascha Kaden
*  \param[in]  mean of distribution
*  \param[out] binary result
*  \date       2016-12-20
*/
bool Sampling::setMeanOfDistribution(const Vec<float> &mean) {
    return m_sampler->setMeanOfDistribution(mean);
}

} /* namespace rmpl */