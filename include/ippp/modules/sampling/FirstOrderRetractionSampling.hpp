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

#ifndef FIRSTORDERRETRACTIONSAMPLING_HPP
#define FIRSTORDERRETRACTIONSAMPLING_HPP

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
class FirstOrderRetractionSampling : public Sampling<dim> {
  public:
    FirstOrderRetractionSampling(const std::shared_ptr<Environment> &environment, const std::shared_ptr<Graph<dim>> &graph,
                                 const std::shared_ptr<Sampler<dim>> &sampler, const std::shared_ptr<Constraint<dim>> &constraint,
                                 const std::shared_ptr<ValidityChecker<dim>> &validityChecker, size_t attempts,
                                 double maxDisplacement, const Transform &taskFrame = Transform::Identity(),
                                 const std::string &name = "FOR-Sampling");

    Vector<dim> getSample() override;
    Vector<dim> getSample(const Vector<dim> &prevSample) override;

  private:
    std::shared_ptr<SerialRobot> m_serialRobot = nullptr;
    std::shared_ptr<Constraint<dim>> m_constraint = nullptr;

    double m_epsilon = 1;
    double m_maxDisplacement;
    Transform m_taskFrame;

    using Sampling<dim>::m_attempts;
    using Sampling<dim>::m_graph;
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
FirstOrderRetractionSampling<dim>::FirstOrderRetractionSampling(const std::shared_ptr<Environment> &environment,
                                                                const std::shared_ptr<Graph<dim>> &graph,
                                                                const std::shared_ptr<Sampler<dim>> &sampler,
                                                                const std::shared_ptr<Constraint<dim>> &constraint,
                                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                                                                size_t attempts, double maxDisplacement,
                                                                const Transform &taskFrame, const std::string &name)
    : Sampling<dim>(name, environment, graph, sampler, validityChecker, attempts),
      m_constraint(constraint),
      m_serialRobot(std::dynamic_pointer_cast<SerialRobot>(environment->getRobot())),
      m_taskFrame(taskFrame),
      m_epsilon(constraint->getEpsilon()),
      m_maxDisplacement(maxDisplacement) {
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
Vector<dim> FirstOrderRetractionSampling<dim>::getSample() {
    Vector<dim> qR = m_sampler->getSample();
    auto vNearVec = m_graph->getNearestNode(qR)->getValues();
    Vector<dim> qS = vNearVec + ((qR - vNearVec) / (qR - vNearVec).norm()) * m_maxDisplacement;

    qR = qS;
    double initDist = (qR - vNearVec).squaredNorm();

    for (size_t i = 0; i < m_attempts; ++i) {
        if (m_constraint->check(qS))
            break;
        Vector6 deltaX = m_constraint->calcEuclideanError(qS);
        auto J = m_serialRobot->calcJacobian(qS);
        J = util::transformToTaskFrameJ(J, m_taskFrame);
        qS -= J.completeOrthogonalDecomposition().pseudoInverse() * deltaX;    // qs -= invJ * deltaX
        if ((qS - qR).squaredNorm() > initDist)
            return util::NaNVector<dim>();
    }

    if (m_validityChecker->check(qS))
        return qS;

    return util::NaNVector<dim>();
}

template <unsigned int dim>
Vector<dim> FirstOrderRetractionSampling<dim>::getSample(const Vector<dim> &prevSample) {
    return getSample();
}

} /* namespace ippp */

#endif /* FIRSTORDERRETRACTIONSAMPLING_HPP */
