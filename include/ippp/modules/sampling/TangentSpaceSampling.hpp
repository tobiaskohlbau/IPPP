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
    TangentSpaceSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<Graph<dim>> &graph,
                         const std::shared_ptr<Sampler<dim>> &sampler,
                         const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts, double maxDisplacement,
                         const std::pair<Vector6, Vector6> &C, const Transform &taskFrame,
                         const std::string &name = "TS-Sampling");

    Vector<dim> getSample() override;

  private:
    std::shared_ptr<SerialRobot> m_serialRobot = nullptr;
    std::shared_ptr<RGDSampling<dim>> m_rgdSampling = nullptr;
    double m_maxDisplacement = 10;
    Matrix<dim> m_I;
    std::pair<Vector6, Vector6> m_C;
    Matrix6 m_matC;
    Transform m_taskFrame;

    using Sampling<dim>::m_graph;
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
                                                const std::shared_ptr<Graph<dim>> &graph,
                                                const std::shared_ptr<Sampler<dim>> &sampler,
                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts,
                                                double maxDisplacement, const std::pair<Vector6, Vector6> &C,
                                                const Transform &taskFrame, const std::string &name)
    : Sampling<dim>(name, environment, graph, sampler, validityChecker, attempts),
      m_maxDisplacement(maxDisplacement),
      m_rgdSampling(std::make_shared<RGDSampling<dim>>(environment, graph, sampler, validityChecker, attempts)),
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
    Vector<dim> qRand = m_sampler->getSample();
    auto vNearVec = m_graph->getNearestNode(qRand)->getValues();
    Vector<dim> qDisplacement = ((qRand - vNearVec) / (qRand - vNearVec).norm()) * m_maxDisplacement;

    auto J = m_serialRobot->calcJacobian(vNearVec);
    J = util::transformToTaskFrameJ(J, m_taskFrame);
    MatrixX invJ = J.completeOrthogonalDecomposition().pseudoInverse();

    Vector<dim> projectedDisplacement = qDisplacement - (m_I - invJ * m_matC * J) * qDisplacement;

    return m_rgdSampling->getSample(vNearVec + projectedDisplacement);
}

} /* namespace ippp */

#endif /* TAGNENTSPACESAMPLING_HPP */
