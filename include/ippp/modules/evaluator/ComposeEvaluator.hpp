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

#ifndef COMPOSEEVALUATOR_HPP
#define COMPOSEEVALUATOR_HPP

#include <ippp/modules/evaluator/Evaluator.hpp>

namespace ippp {

/*!
* \brief   The ComposeEvaluator combines different Evaluators by AND or OR.
* \details At a initialization step it init all owned evaluators and all poses and configurations will be passed to the owned
* evaluators.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class ComposeEvaluator : public Evaluator<dim> {
  public:
    ComposeEvaluator(const std::vector<std::shared_ptr<Evaluator<dim>>> evaluators, ComposeType type,
                     const std::string &name = "ComposeEvaluator");

    bool evaluate();
    void initialize() override;
    virtual void setConfigs(const std::vector<Vector<dim>> &targets) override;
    virtual void setPoses(const std::vector<Vector6> &targets) override;

  protected:
    std::vector<std::shared_ptr<Evaluator<dim>>> m_evaluators;
    const ComposeType m_type = ComposeType::AND;
};

/*!
*  \brief      Constructor of the class ComposeEvaluator
*  \author     Sascha Kaden
*  \param[in]  List of Evaluator
*  \param[in]  type of composing
*  \date       2017-09-30
*/
template <unsigned int dim>
ComposeEvaluator<dim>::ComposeEvaluator(const std::vector<std::shared_ptr<Evaluator<dim>>> evaluators, ComposeType type,
                                        const std::string &name)
    : Evaluator<dim>(name), m_evaluators(evaluators), m_type(type) {
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

template <unsigned int dim>
void ComposeEvaluator<dim>::initialize() {
    Logging::debug("Initialize", this);
    for (auto &evaluator : m_evaluators)
        evaluator->initialize();
}

/*!
*  \brief      Set target configurations for evaluation to the owned list of evaluators.
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
*  \brief      Set target poses for evaluation to the owned list of evaluators.
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
