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

#ifndef DISTANCEMETRIC_HPP
#define DISTANCEMETRIC_HPP

#include <memory>

#include <core/Identifier.h>
#include <core/dataObj/Node.hpp>
#include <core/types.h>

namespace ippp {

template <unsigned int dim>
class Node;

/*!
* \brief   Virtual class for the computation of distance costs from an Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class DistanceMetric : public Identifier {
  public:
    DistanceMetric(const std::string &name = "L2 Metric");
    double calcDist(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target) const;
    virtual double calcDist(const Vector<dim> &source, const Vector<dim> &target) const = 0;
};

/*!
*  \brief      Standard constructor of the class DistanceMetric
*  \author     Sascha Kaden
*  \param[in]  name
*  \date       2017-02-19
*/
template <unsigned int dim>
DistanceMetric<dim>::DistanceMetric(const std::string &name) : Identifier(name) {
}

/*!
*  \brief      Calculates the distance cost of an Edge from the source and target Node.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] distance cost
*  \date       2017-01-02
*/
template <unsigned int dim>
double DistanceMetric<dim>::calcDist(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target) const {
    return calcDist(source->getValues(), target->getValues());
}

} /* namespace ippp */

#endif    // DISTANCEMETRIC_HPP
