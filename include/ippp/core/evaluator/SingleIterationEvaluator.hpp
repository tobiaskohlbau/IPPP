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

#ifndef DUMMYEVALUATOR_HPP
#define DUMMYEVALUATOR_HPP

#include <ippp/core/evaluator/Evaluator.hpp>

namespace ippp {

/*!
* \brief   SingleIterationEvaluator which runs only one Iteration.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class SingleIterationEvaluator : public Evaluator<dim> {
  public:
    SingleIterationEvaluator(const std::shared_ptr<Environment> &environment);

    bool evaluate();

  protected:
    bool m_firstEvaluation = true;
};

/*!
*  \brief      Constructor of the class SingleIterationEvaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-09-30
*/
template <unsigned int dim>
SingleIterationEvaluator<dim>::SingleIterationEvaluator(const std::shared_ptr<Environment> &environment)
    : Evaluator<dim>("SingleIterationEvaluator", environment) {
}

/*!
*  \brief      Return always true at the second iteration
*  \author     Sascha Kaden
*  \param[out] evaluation
*  \date       2017-09-30
*/
template <unsigned int dim>
bool SingleIterationEvaluator<dim>::evaluate() {
    if (m_firstEvaluation) {
        m_firstEvaluation = false;
        return false;
    } else {
        return true;
    }
}

} /* namespace ippp */

#endif /* DUMMYEVALUATOR_HPP */
