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

#include <ippp/modules/distanceMetrics/DistanceMetric.hpp>

namespace ippp {

/*!
* \brief   WeightedL2Metric computes the distance of nodes/configurations with the L2 metric and a weighted vector.
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class WeightedL2Metric : public DistanceMetric<dim> {
  public:
    WeightedL2Metric();
    WeightedL2Metric(const Vector<dim> &weightVec);
    double calcDist(const Vector<dim> &source, const Vector<dim> &target) const override;
    double calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const override;
    void simplifyDist(double &dist) const override;

    void setWeightVec(const Vector<dim> &weight);
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
WeightedL2Metric<dim>::WeightedL2Metric() : DistanceMetric<dim>("weightVecL2 metric") {
}

/*!
*  \brief      Constructor of the class WeightVecL2Metric
*  \author     Sascha Kaden
*  \param[in]  vector of the weights
*  \date       2017-02-19
*/
template <unsigned int dim>
WeightedL2Metric<dim>::WeightedL2Metric(const Vector<dim> &weightVec) : DistanceMetric<dim>("weightVecL2 metric") {
    setWeightVec(weightVec);
}

/*!
*  \brief      Calculates the distance cost of an Edge from the source and target Node by the specified metric.
*  \author     Sascha Kaden
*  \param[in]  source vector
*  \param[in]  target vector
*  \param[out] distance cost
*  \date       2017-01-02
*/
template <unsigned int dim>
double WeightedL2Metric<dim>::calcDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).norm();
}

/*!
*  \brief      Calculates the squared distance cost of an Edge from the source and target Node by the specified metric.
*  \author     Sascha Kaden
*  \param[in]  source vector
*  \param[in]  target vector
*  \param[out] squared distance cost
*  \date       2017-10-08
*/
template <unsigned int dim>
double WeightedL2Metric<dim>::calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).squaredNorm();
}

/*!
*  \brief      Calculates the simplified distance of the passed distance.
*  \author     Sascha Kaden
*  \param[in]  distance
*  \param[out] simplified distance
*  \date       2017-10-08
*/
template <unsigned int dim>
void WeightedL2Metric<dim>::simplifyDist(double &dist) const {
    dist *= dist;
}

/*!
*  \brief      Sets the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
void WeightedL2Metric<dim>::setWeightVec(const Vector<dim> &weight) {
    m_weightVec = weight;
}

/*!
*  \brief      Returns the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
Vector<dim> WeightedL2Metric<dim>::getWeightVec() const {
    return m_weightVec;
}

} /* namespace ippp */

#endif    // WEIGHTVECL2METRIC_HPP
