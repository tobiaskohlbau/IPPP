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

#ifndef EVALUATOR_HPP
#define EVALUATOR_HPP

#include <ippp/core/Identifier.h>
#include <ippp/core/types.h>
#include <ippp/environment/Environment.h>
#include <ippp/core/util/UtilVec.hpp>

namespace ippp {

/*!
* \brief   Evaluator interface.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class Evaluator : public Identifier {
  public:
    Evaluator(const std::string &name, const std::shared_ptr<Environment> &environment);

    virtual bool evaluate() = 0;
    virtual void setQuery(const std::vector<Vector<dim>> &targets);

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;
    std::vector<Vector<dim>> m_targets;
};

/*!
*  \brief      Constructor of the class Evaluator
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \date       2017-09-30
*/
template <unsigned int dim>
Evaluator<dim>::Evaluator(const std::string &name, const std::shared_ptr<Environment> &environment)
    : Identifier(name), m_environment(environment) {
}

/*!
*  \brief      Set target nodes for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void Evaluator<dim>::setQuery(const std::vector<Vector<dim>> &targets) {
    if (targets.empty())
        return;

    for (auto &target : targets)
        if (util::empty<dim>(target))
            return;

    m_targets = targets;
}

} /* namespace ippp */

#endif /* EVALUATOR_HPP */
