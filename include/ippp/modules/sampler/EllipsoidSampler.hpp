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

#ifndef ELLIPSOIDSAMPLER_HPP
#define ELLIPSOIDSAMPLER_HPP

#include <Eigen/SVD>

#include <ippp/modules/distanceMetrics/L2Metric.hpp>
#include <ippp/modules/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   Class SamplerUniform creates uniform samples, area is from the robot boundaries defined.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class EllipsoidSampler : public Sampler<dim> {
  public:
    EllipsoidSampler(const std::shared_ptr<Environment> &environment, const std::string &seed = "");
    EllipsoidSampler(const std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed = "");
    Vector<dim> getSample() override;

    void setParams(const Vector<dim> &start, const Vector<dim> &goal, double cMax);

  private:
    std::vector<std::uniform_real_distribution<double>> m_distUniform;
    Vector<dim> m_start, m_goal, m_centre;
    double m_cMin, m_cMax;
    Matrix<dim> m_C, m_L, m_CL;
    L2Metric<dim> m_metric;

    using Sampler<dim>::m_robotBoundary;
    using Sampler<dim>::m_generator;
};

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
EllipsoidSampler<dim>::EllipsoidSampler(const std::shared_ptr<Environment> &environment, const std::string &seed)
    : Sampler<dim>("SamplerUniform", environment, seed) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(-1, 1);
        m_distUniform.push_back(dist);
    }
}

/*!
*  \brief      Constructor of the class SamplerUniform
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
EllipsoidSampler<dim>::EllipsoidSampler(std::pair<Vector<dim>, Vector<dim>> boundary, const std::string &seed)
    : Sampler<dim>("SamplerUniform", boundary, seed) {
    for (unsigned int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> dist(-1, 1);
        m_distUniform.push_back(dist);
    }
}

/*!
*  \brief      Return uniform sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> EllipsoidSampler<dim>::getSample() {
    Vector<dim> config;

    do {
        for (unsigned int i = 0; i < dim; ++i)
            config[i] = m_distUniform[i](m_generator);
    } while (config.squaredNorm() > 1);

    // check that config is inside ball

    // check that config is inside ellipsoid

    Vector<dim> tmp = m_CL * config;
    Vector<dim> result = m_centre + tmp;
    return result;

    // return config;
}

template <unsigned int dim>
void EllipsoidSampler<dim>::setParams(const Vector<dim> &start, const Vector<dim> &goal, double cMax) {
    m_start = start;
    m_goal = goal;
    m_cMax = cMax;

    m_cMin = m_metric.calcDist(start, goal);
    m_centre = (m_start + m_goal) / 2;

    Vector<dim> a1 = (m_goal - m_start) / m_cMin;
    Vector<dim> firstColoumIdentity = Vector<dim>::Zero();
    firstColoumIdentity[0] = 1;
    Matrix<dim> M = a1 * firstColoumIdentity.transpose();
    Eigen::JacobiSVD<Matrix<dim>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector<dim> ones = Vector<dim>::Constant(1);
    ones[dim - 2] = svd.matrixU().determinant();
    ones[dim - 1] = svd.matrixV().determinant();
    
    m_C = svd.matrixU() * ones.asDiagonal() * svd.matrixV();

    Vector<dim> r = Vector<dim>::Constant((std::sqrt((m_cMax * m_cMax) - (m_cMin * m_cMin))) / 2);
    r[0] = m_cMax / 2;
    m_L = r.asDiagonal();

    m_CL = m_C * m_L;
}

} /* namespace ippp */

#endif /* ELLIPSOIDSAMPLER_HPP */
