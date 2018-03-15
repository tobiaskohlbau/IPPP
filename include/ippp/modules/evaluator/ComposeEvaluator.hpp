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

#ifndef COMPOSEEVALUATOR_HPP
#define COMPOSEEVALUATOR_HPP

#include <ippp/modules/evaluator/Evaluator.hpp>

namespace ippp {

/*!
* \brief   ComposeEvaluator which runs only one Iteration.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class ComposeEvaluator : public Evaluator<dim> {
  public:
    ComposeEvaluator(const std::vector<std::shared_ptr<Evaluator<dim>>> evaluators, ComposeType type);

    bool evaluate();
    virtual void setConfigs(const std::vector<Vector<dim>> &targets) override;
    virtual void setPoses(const std::vector<Vector6> &targets) override;

  protected:
    std::vector<std::shared_ptr<Evaluator<dim>>> m_evaluators;
    const ComposeType m_type = ComposeType::AND;
};

/*!
*  \brief      Constructor of the class SingleIterationEvaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-09-30
*/
template <unsigned int dim>
ComposeEvaluator<dim>::ComposeEvaluator(const std::vector<std::shared_ptr<Evaluator<dim>>> evaluators, ComposeType type)
    : Evaluator<dim>("ComposeEvaluator"), m_evaluators(evaluators), m_type(type) {
}

/*!
*  \brief      Return always true at the second iteration
*  \author     Sascha Kaden
*  \param[out] evaluation
*  \date       2017-09-30
*/
template <unsigned int dim>
bool ComposeEvaluator<dim>::evaluate() {
    if (m_type == ComposeType::AND) {
        for (auto &evaluator : m_evaluators)
            if (!evaluator->evaluate())
                return false;

        return true;
    } else {    // OR
        for (auto &evaluator : m_evaluators)
            if (evaluator->evaluate())
                return true;

        return false;
    }
}

/*!
*  \brief      Set target nodes for evaluation inside list of evaluators.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void ComposeEvaluator<dim>::setConfigs(const std::vector<Vector<dim>> &targets) {
    for (auto &evaluator : m_evaluators)
        evaluator->setConfigs(targets);
}

/*!
*  \brief      Set target nodes for evaluation inside list of evaluators.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void ComposeEvaluator<dim>::setPoses(const std::vector<Vector6> &targets) {
    for (auto &evaluator : m_evaluators)
        evaluator->setPoses(targets);
}

} /* namespace ippp */

#endif /* COMPOSEEVALUATOR_HPP */
