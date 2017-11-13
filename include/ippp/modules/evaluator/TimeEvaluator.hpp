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

#ifndef TIMEEVALUATOR_HPP
#define TIMEEVALUATOR_HPP

#include <chrono>

#include <ippp/modules/evaluator/Evaluator.hpp>

namespace ippp {

/*!
* \brief   SingleIterationEvaluator which runs only one Iteration.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class TimeEvaluator : public Evaluator<dim> {
  public:
    TimeEvaluator(const size_t maxDuration = 30);

    bool evaluate();

  protected:
    size_t m_maxDuration;
    bool m_started = false;
    std::chrono::time_point<std::chrono::system_clock> m_startTime;
};

/*!
*  \brief      Constructor of the class SingleIterationEvaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-09-30
*/
template <unsigned int dim>
TimeEvaluator<dim>::TimeEvaluator(const size_t maxDuration)
    : Evaluator<dim>("SingleIterationEvaluator"), m_maxDuration(maxDuration) {
}

/*!
*  \brief      Return always true at the second iteration
*  \author     Sascha Kaden
*  \param[out] evaluation
*  \date       2017-09-30
*/
template <unsigned int dim>
bool TimeEvaluator<dim>::evaluate() {
    if (!m_started) {
        m_startTime = std::chrono::system_clock::now();
        m_started = true;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_startTime);
    if (static_cast<size_t>(duration.count()) > m_maxDuration * 1000) {
        Logging::info(
            "Maximum duration achieved with: " + std::to_string(static_cast<double>(duration.count() / 1000)) + " seconds.",
            this);
        return true;
    } else {
        return false;
    }
}

} /* namespace ippp */

#endif /* TIMEEVALUATOR_HPP */
