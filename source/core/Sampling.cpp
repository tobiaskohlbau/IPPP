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

#include <core/Sampling.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \date       2016-05-24
*/
Sampling::Sampling(const std::shared_ptr<RobotBase> &robot, SamplingMethod method) : Base("Sampling") {
    m_method = method;
    m_robot = robot;

    m_minBoundary = m_robot->getMinBoundary();
    m_maxBoundary = m_robot->getMaxBoundary();

    srand(time(NULL));
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  index from loop of samples
*  \param[in]  absolute number of samples, which will be created
*  \param[out] Vec
*  \date       2016-05-24
*/
Vec<float> Sampling::getSample(unsigned int dim, int index, int nbSamples) {
    Vec<float> vec(dim);
    if (!checkBoudaries())
        return vec;

    if (true)    // m_method == SamplingMethod::randomly)
    {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_minBoundary[i] + (float)(rand() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
        return vec;
    }
    return vec;
}

/*!
*  \brief      Check boundary existence
*  \author     Sascha Kaden
*  \param[out] result of check
*  \date       2016-05-24
*/
bool Sampling::checkBoudaries() {
    if (m_minBoundary.empty() || m_maxBoundary.empty()) {
        this->sendMessage("Boundaries are empty", Message::error);
        return false;
    }
    return true;
}
