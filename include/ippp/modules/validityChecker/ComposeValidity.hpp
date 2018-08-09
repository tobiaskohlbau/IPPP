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

#ifndef COMPOSEVALIDITY_HPP
#define COMPOSEVALIDITY_HPP

#include <ippp/modules/validityChecker/ValidityChecker.hpp>

namespace ippp {

/*!
* \brief   Base class of all ValidityChecker.
* \author  Sascha Kaden
* \date    2018-01-17
*/
template <unsigned int dim>
class ComposeValidity : public ValidityChecker<dim> {
  public:
    ComposeValidity(const std::shared_ptr<Environment> &environment,
                    const std::vector<std::shared_ptr<ValidityChecker<dim>>> &checker, ComposeType type);

    bool check(const Vector<dim> &config) const;
    bool check(const std::vector<Vector<dim>> &configs) const;
    virtual double calc(const Vector<dim> &config) const override;
    virtual double calc(const std::vector<Vector<dim>> &config) const override;

    std::shared_ptr<ValidityChecker<dim>> getChecker(size_t index);

  protected:
    std::vector<std::shared_ptr<ValidityChecker<dim>>> m_checkers;
    const ComposeType m_type = ComposeType::AND;
};

/*!
*  \brief      Constructor of the class ValidityChecker
*  \param[in]  name
*  \param[in]  Environment
*  \author     Sascha Kaden
*  \date       2018-01-17
*/
template <unsigned int dim>
ComposeValidity<dim>::ComposeValidity(const std::shared_ptr<Environment> &environment,
                                      const std::vector<std::shared_ptr<ValidityChecker<dim>>> &checker, ComposeType type)
    : ValidityChecker<dim>("ComposeValidity", environment), m_checkers(checker), m_type(type) {
}

/*!
*  \brief      Calculates the validity of the passed configuration, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ComposeValidity<dim>::check(const Vector<dim> &config) const {
    if (m_type == ComposeType::AND) {
        for (auto &checker : m_checkers)
            if (!checker->check(config))
                return false;

        return true;
    } else {    // OR
        for (auto &checker : m_checkers)
            if (checker->check(config))
                return true;

        return false;
    }
}

/*!
*  \brief      Calculates the validity of the passed configurations, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ComposeValidity<dim>::check(const std::vector<Vector<dim>> &configs) const {
    for (auto &config : configs)
        if (!check(config))
            return false;
    return true;
}

template <unsigned int dim>
double ComposeValidity<dim>::calc(const Vector<dim> &config) const {
    double error = 0;
    double tmpError;
    for (auto &checker : m_checkers) {
        tmpError = checker->calc(config);
        if (tmpError > error)
            error = tmpError;
    }
    return error;
}

template <unsigned int dim>
double ComposeValidity<dim>::calc(const std::vector<Vector<dim>> &configs) const {
    double error = 0;
    double tmpError;
    for (auto &config : configs) {
        for (auto &checker : m_checkers) {
            tmpError = checker->calc(config);
            if (tmpError > error)
                error = tmpError;
        }
    }
    return error;
}

template <unsigned int dim>
std::shared_ptr<ValidityChecker<dim>> ComposeValidity<dim>::getChecker(size_t index) {
    if (index >= m_checkers.size()) {
        Logging::error("Index to large!", this);
        return nullptr;
    }
    return m_checkers[index];
}

} /* namespace ippp */

#endif /* COMPOSEVALIDITY_HPP */