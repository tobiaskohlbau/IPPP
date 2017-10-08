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

#ifndef L2METRIC_HPP
#define L2METRIC_HPP

#include <ippp/core/distanceMetrics/DistanceMetric.hpp>

namespace ippp {

/*!
* \brief   Static class for the computation of distance costs from Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class L2Metric : public DistanceMetric<dim> {
  public:
    L2Metric();
    double calcDist(const Vector<dim> &source, const Vector<dim> &target) const;
    double calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const;
    void simplifyDist(double &dist) const;
};

/*!
*  \brief      Standard constructor of the class L2Metric.
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
template <unsigned int dim>
L2Metric<dim>::L2Metric() : DistanceMetric<dim>("L2 Metric") {
}

/*!
*  \brief      Calculates the distance cost of an Edge from the source and target Node by the L2 metric.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] distance cost
*  \date       2017-01-02
*/
template <unsigned int dim>
double L2Metric<dim>::calcDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).norm();
}

/*!
*  \brief      Calculates the squared distance cost of an Edge from the source and target Node by the L2 metric.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] squared distance cost
*  \date       2017-10-08
*/
template <unsigned int dim>
double L2Metric<dim>::calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).squaredNorm();
}

/*!
*  \brief      Calculates the simplified distance of the passed distance.
*  \author     Sascha Kaden
*  \param[in]  distance
*  \param[out] simplified distance
*  \date       2017-10-08
*/
template <unsigned int dim>
void L2Metric<dim>::simplifyDist(double &dist) const {
    dist *= dist;
}

} /* namespace ippp */

#endif    // L2METRIC_HPP
