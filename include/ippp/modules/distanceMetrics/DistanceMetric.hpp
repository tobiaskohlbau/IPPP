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

#ifndef DISTANCEMETRIC_HPP
#define DISTANCEMETRIC_HPP

#include <memory>

#include <ippp/Identifier.h>
#include <ippp/dataObj/Node.hpp>
#include <ippp/types.h>
#include <ippp/util/Logging.h>

namespace ippp {

template <unsigned int dim>
class Node;

/*!
* \brief   Interface class for the computation of the distance or cost between two nodes/configurations.
* \details If the subclass supports it, a simplified computation is provided, otherwise the normal computation will be done.
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class DistanceMetric : public Identifier {
  public:
    DistanceMetric(const std::string &name);
    double calcDist(const Node<dim> &source, const Node<dim> &target) const;
    double calcSimpleDist(const Node<dim> &source, const Node<dim> &target) const;
    virtual double calcDist(const Vector<dim> &source, const Vector<dim> &target) const = 0;
    virtual double calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const;
    virtual void simplifyDist(double &dist) const;
};

/*!
*  \brief      Standard constructor of the class DistanceMetric
*  \author     Sascha Kaden
*  \param[in]  name
*  \date       2017-02-19
*/
template <unsigned int dim>
DistanceMetric<dim>::DistanceMetric(const std::string &name) : Identifier(name) {
    Logging::debug("Initialize", this);
}

/*!
*  \brief      Calculates the distance cost between two Nodes.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] distance cost
*  \date       2017-01-02
*/
template <unsigned int dim>
double DistanceMetric<dim>::calcDist(const Node<dim> &source, const Node<dim> &target) const {
    return calcDist(source.getValues(), target.getValues());
}

/*!
*  \brief      Calculates the simplified distance cost between two Nodes.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] simplified distance cost
*  \date       2017-10-08
*/
template <unsigned int dim>
double DistanceMetric<dim>::calcSimpleDist(const Node<dim> &source, const Node<dim> &target) const {
    return calcSimpleDist(source.getValues(), target.getValues());
}

/*!
*  \brief      Calculates the simplified distance cost between two configurations.
*  \author     Sascha Kaden
*  \param[in]  source configuration
*  \param[in]  target configuration
*  \param[out] simplified distance cost
*  \date       2018-07-25
*/
template <unsigned int dim>
double DistanceMetric<dim>::calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return calcDist(source, target);
}

/*!
*  \brief         Calculates the simplified distance of the passed distance (do nothing).
*  \author        Sascha Kaden
*  \param[in,out] distance
*  \date          2018-07-25
*/
template <unsigned int dim>
void DistanceMetric<dim>::simplifyDist(double &dist) const {
}

} /* namespace ippp */

#endif    // DISTANCEMETRIC_HPP
