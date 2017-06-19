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

#ifndef INFMETRIC_HPP
#define INFMETRIC_HPP

#include <ippp/core/distanceMetrics/DistanceMetric.hpp>

namespace ippp {

/*!
* \brief   Static class for the computation of distance costs from Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class InfMetric : public DistanceMetric<dim> {
  public:
    InfMetric();
    double calcDist(const Vector<dim> &source, const Vector<dim> &target) const override;
};

/*!
*  \brief      Standard constructor of the class InfMetric
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
template <unsigned int dim>
InfMetric<dim>::InfMetric() : DistanceMetric<dim>("Inf Metric") {
}

/*!
*  \brief      Calculates the distance cost of an Edge from the source and target Node by the specified metric.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] distance cost
*  \date       2017-01-02
*/
template <unsigned int dim>
double InfMetric<dim>::calcDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).maxCoeff();
}

} /* namespace ippp */

#endif    // INFMETRIC_HPP
