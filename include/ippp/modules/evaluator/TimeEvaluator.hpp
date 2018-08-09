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

#ifndef TIMEEVALUATOR_HPP
#define TIMEEVALUATOR_HPP

#include <chrono>

#include <ippp/modules/evaluator/Evaluator.hpp>

namespace ippp {

/*!
* \brief   TimeEvaluator returns true after the defined time.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class TimeEvaluator : public Evaluator<dim> {
  public:
    TimeEvaluator(size_t maxDuration = 30, const std::string &name = "TimeEvaluator");

    bool evaluate();
    void initialize() override;

  protected:
    size_t m_maxDuration;
    bool m_started = false;
    std::chrono::time_point<std::chrono::system_clock> m_startTime;
};

template <unsigned int dim>
TimeEvaluator<dim>::TimeEvaluator(size_t maxDuration, const std::string &name)
    : Evaluator<dim>(name), m_maxDuration(maxDuration) {
}

/*!
*  \brief      Return true after the defined time has passed.
*  \author     Sascha Kaden
*  \param[out] evaluation result
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
    }
    return false;
}

template <unsigned int dim>
void TimeEvaluator<dim>::initialize() {
    Logging::debug("Initialize", this);
    m_started = false;
}

} /* namespace ippp */

#endif /* TIMEEVALUATOR_HPP */
