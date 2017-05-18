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

#ifndef WEIGHTVECL2METRIC_HPP
#define WEIGHTVECL2METRIC_HPP

#include <core/distanceMetrics/DistanceMetric.hpp>

namespace ippp {

/*!
* \brief   Static class for the computation of distance costs from Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class WeightVecL2Metric : public DistanceMetric<dim> {
  public:
    WeightVecL2Metric();
    WeightVecL2Metric(const Vector<dim> &weightVec);
    float calcDist(const Vector<dim> &source, const Vector<dim> &target) const override;

    void setWeightVec(const Vector<dim> &vec);
    Vector<dim> getWeightVec() const;

  private:
    Vector<dim> m_weightVec;
};

/*!
*  \brief      Standard constructor of the class WeightVecL2Metric
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
template <unsigned int dim>
WeightVecL2Metric<dim>::WeightVecL2Metric() : DistanceMetric<dim>("weightVecL2 metric") {
}

/*!
*  \brief      Constructor of the class WeightVecL2Metric
*  \author     Sascha Kaden
*  \param[in]  weightVec
*  \date       2017-02-19
*/
template <unsigned int dim>
WeightVecL2Metric<dim>::WeightVecL2Metric(const Vector<dim> &weightVec) : DistanceMetric<dim>("weightVecL2 metric") {
    setWeightVec(weightVec);
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
float WeightVecL2Metric<dim>::calcDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).norm();
}

/*!
*  \brief      Sets the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
void WeightVecL2Metric<dim>::setWeightVec(const Vector<dim> &vec) {
    m_weightVec = vec;
}

/*!
*  \brief      Returns the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
Vector<dim> WeightVecL2Metric<dim>::getWeightVec() const {
    return m_weightVec;
}

} /* namespace ippp */

#endif    // WEIGHTVECL2METRIC_HPP
