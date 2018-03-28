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

#ifndef TAGNENTSPACESAMPLING_HPP
#define TAGNENTSPACESAMPLING_HPP

#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/modules/sampler/SamplerNormalDist.hpp>
#include <ippp/modules/sampling/RGDSampling.hpp>
#include <ippp/modules/sampling/Sampling.hpp>

namespace ippp {

/*!
* \brief   Class SamplingNearObstacle creates samples with the passed Sampler and if they in collision valid samples in
* neighborhood will found.
* \author  Sascha Kaden
* \date    2016-12-20
*/
template <unsigned int dim>
class TangentSpaceSampling : public Sampling<dim> {
  public:
    TangentSpaceSampling(const std::shared_ptr<Environment> &environment,
                         const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                         const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts, const std::shared_ptr<Graph<dim>> &graph,
                         double maxDisplacement, const std::pair<Vector6, Vector6> &C, const Transform &taskFrame);

    Vector<dim> getSample() override;
    Vector<dim> getSample(const Vector<dim> &prevSample) override;

  private:
    std::shared_ptr<SerialRobot> m_serialRobot = nullptr;
    std::shared_ptr<RGDSampling<dim>> m_rgdSampling = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    double m_maxDisplacement = 10;
    Matrix<dim> m_I;
    std::pair<Vector6, Vector6> m_C;
    Matrix6 m_matC;
    Transform m_taskFrame;

    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_sampler;
    using Sampling<dim>::m_validityChecker;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \param[in]  Sampler
*  \param[in]  attempts for one sampling
*  \date       2016-12-20
*/
template <unsigned int dim>
TangentSpaceSampling<dim>::TangentSpaceSampling(const std::shared_ptr<Environment> &environment,
                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                                                const std::shared_ptr<Sampler<dim>> &sampler, size_t attempts,
                                                const std::shared_ptr<Graph<dim>> &graph, double maxDisplacement,
                                                const std::pair<Vector6, Vector6> &C, const Transform &taskFrame)
    : Sampling<dim>("TangentSpaceSampling", environment, validityChecker, sampler, attempts),
      m_maxDisplacement(maxDisplacement),
      m_rgdSampling(std::make_shared<RGDSampling<dim>>(environment, validityChecker, sampler, attempts, graph)),
      m_graph(graph),
      m_serialRobot(std::dynamic_pointer_cast<SerialRobot>(environment->getRobot())),
      m_I(Matrix<dim>::Identity()),
      m_C(C),
      m_taskFrame(taskFrame) {
    m_matC = Matrix6::Zero();
    for (unsigned int i = 0; i < 6; ++i) {
        if (m_C.first[i] == -IPPP_MAX && m_C.second[i] == IPPP_MAX)
            continue;

        m_matC(i, i) = 1;
    }
}

/*!
*  \brief      Sample in the neighborhood of obstacles
*  \details    If Sample is in collision, second random collision valid sample will be computed and by binary search the
*              nearest collision valid sample to the first sample, will be taken.
*  \author     Sascha Kaden
*  \param[out] sample Vector
*  \date       2016-12-20
*/
template <unsigned int dim>
Vector<dim> TangentSpaceSampling<dim>::getSample() {
    return getSample(m_graph->getNode(static_cast<size_t>(this->getRandomNumber() * m_graph->numNodes()))->getValues());
}

template <unsigned int dim>
Vector<dim> TangentSpaceSampling<dim>::getSample(const Vector<dim> &prevSample) {
    Vector<dim> displacement = m_sampler->getRandomRay() * m_sampler->getRandomNumber() * m_maxDisplacement;

    auto J = m_serialRobot->calcJacobian(prevSample);
    J = util::transformToTaskFrameJ(J, m_taskFrame);
    MatrixX invJ = J.completeOrthogonalDecomposition().pseudoInverse();

    Vector<dim> projectedDisplacement = (m_I - invJ * m_matC * J) * displacement;

    return m_rgdSampling->getSample(prevSample + projectedDisplacement);
}

} /* namespace ippp */

#endif /* TAGNENTSPACESAMPLING_HPP */
