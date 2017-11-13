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

#ifndef GRIDSAMPLER_HPP
#define GRIDSAMPLER_HPP

#include <mutex>

#include <ippp/modules/sampler/Sampler.hpp>

namespace ippp {

/*!
* \brief   GridSampler, creates uniform grid samples between the boudaries of the robot.
* \details All samples will generated at the construction of the module.
* \author  Sascha Kaden
* \date    2017-11-13
*/
template <unsigned int dim>
class GridSampler : public Sampler<dim> {
  public:
    GridSampler(const std::shared_ptr<Environment> &environment, const double res = 1);
    GridSampler(const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary, const double res = 1);
    void setResolution(const double res);
    Vector<dim> getSample();

  protected:
    void generateGridConfigs();
    void fillConfig(Vector<dim> values, const unsigned int index);

    double m_res = 1;
    size_t m_curIndex = 0;
    std::vector<Vector<dim>> m_gridConfigs;
    std::mutex m_mutex;

    using Sampler<dim>::m_minBoundary;
    using Sampler<dim>::m_maxBoundary;
    using Sampler<dim>::m_generator;
};

/*!
*  \brief      Constructor of the GridSampler class
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-11-13
*/
template <unsigned int dim>
GridSampler<dim>::GridSampler(const std::shared_ptr<Environment> &environment, const double res)
    : Sampler<dim>("GridSampler", environment, std::string()) {
    setResolution(res);
    generateGridConfigs();
}

/*!
*  \brief      Constructor of the RandomSampler class
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \date       2017-11-13
*/
template <unsigned int dim>
GridSampler<dim>::GridSampler(const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary, const double res)
    : Sampler<dim>("RandomSampler", minBoundary, maxBoundary, , std::string()) {
    setResolution(res);
    generateGridConfigs();
}

template <unsigned int dim>
void GridSampler<dim>::setResolution(const double res) {
    if (res <= 0)
        Logging::error("Resolution has to be larger than 0!", this);
    else
        m_res = res;
}

/*!
*  \brief      Return random sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> GridSampler<dim>::getSample() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_gridConfigs.empty())
        return Vector<dim>();

    if (m_curIndex < m_gridConfigs.size()) {
        ++m_curIndex;
        return m_gridConfigs[m_curIndex - 1];
    }
    m_curIndex = 1;
    return m_gridConfigs.front();
}

/*!
*  \brief      Generate all grid configs to the defined resolution
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
void GridSampler<dim>::generateGridConfigs() {
    m_gridConfigs.clear();

    fillConfig(Vector<dim>(), 0);
}

template <unsigned int dim>
void GridSampler<dim>::fillConfig(Vector<dim> values, unsigned int index) {
    if (index == dim - 1) {
        for (double value = m_minBoundary[index]; value < m_maxBoundary[index]; value += m_res) {
            Vector<dim> config = values;
            config[index] = value;
            m_gridConfigs.push_back(config);
        }
    } else {
        for (double value = m_minBoundary[index]; value < m_maxBoundary[index]; value += m_res) {
            values[index] = value;
            fillConfig(values, index + 1);
        }
    }
}

} /* namespace ippp */

#endif /* GRIDSAMPLER_HPP */
