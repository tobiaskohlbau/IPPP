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

#ifndef WEIGHTVECINFMETRIC_HPP
#define WEIGHTVECINFMETRIC_HPP

#include <ippp/modules/distanceMetrics/DistanceMetric.hpp>

namespace ippp {

/*!
* \brief   Static class for the computation of distance costs from an Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class WeightedInfMetric : public DistanceMetric<dim> {
  public:
    WeightedInfMetric();
    WeightedInfMetric(const Vector<dim> &weightVec);
    double calcDist(const Vector<dim> &source, const Vector<dim> &target) const;
    double calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const;
    void simplifyDist(double &dist) const;

    void setWeightVec(const Vector<dim> &weight);
    Vector<dim> getWeightVec() const;

  private:
    Vector<dim> m_weightVec;
};

/*!
*  \brief      Standard constructor of the class WeightVecInfMetric
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
template <unsigned int dim>
WeightedInfMetric<dim>::WeightedInfMetric() : DistanceMetric<dim>("weightVecInf metric") {
}

/*!
*  \brief      Standard constructor of the class WeightVecInfMetric
*  \author     Sascha Kaden
*  \param[in]  weightVec
*  \date       2017-02-19
*/
template <unsigned int dim>
WeightedInfMetric<dim>::WeightedInfMetric(const Vector<dim> &weightVec) : DistanceMetric<dim>("weightVecInf metric") {
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
double WeightedInfMetric<dim>::calcDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).maxCoeff();
}

/*!
*  \brief      Calculates the squared distance cost of an Edge from the source and target Node by the specified metric.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] squared distance cost
*  \date       2017-10-08
*/
template <unsigned int dim>
double WeightedInfMetric<dim>::calcSimpleDist(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).maxCoeff();
}

/*!
*  \brief      Calculates the simplified distance of the passed distance.
*  \author     Sascha Kaden
*  \param[in]  distance
*  \param[out] simplified distance
*  \date       2017-10-08
*/
template <unsigned int dim>
void WeightedInfMetric<dim>::simplifyDist(double &dist) const {
}

/*!
*  \brief      Sets the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
void WeightedInfMetric<dim>::setWeightVec(const Vector<dim> &weight) {
    m_weightVec = weight;
}

/*!
*  \brief      Returns the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
Vector<dim> WeightedInfMetric<dim>::getWeightVec() const {
    return m_weightVec;
}

} /* namespace ippp */

#endif    // WEIGHTVECINFMETRIC_HPP
