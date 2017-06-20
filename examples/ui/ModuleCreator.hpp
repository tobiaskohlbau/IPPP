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

#ifndef MODULECREATOR_HPP
#define MODULECREATOR_HPP

#include <fstream>
#include <type_traits>
#include <vector>

#include <ippp/Core>
#include <ippp/Environment>
#include <ippp/Planner>

namespace ippp {

enum class MetricType { L1, L2, Inf };

enum class NeighborType { KDTree, BruteForce };

enum class PathModifierType { Dummy, NodeCut };

enum class SamplerType { SamplerRandom, SamplerNormalDist, SamplerUniform, SeedSampler };

enum class SamplingType { Bridge, Gaussian, GaussianDist, Straight, MedialAxis, NearObstacle };

enum class TrajectoryType { Linear, RotateAtS};

/*!
* \brief   Class ModuleCreator generates all defined modules for the path planner and creates the graph for the planner too. By a
* method it returns the options for the planner.
* \author  Sascha Kaden
* \date    2017-05-22
*/
template <unsigned int dim>
class ModuleCreator : public Identifier {
  public:
    ModuleCreator(std::shared_ptr<Environment> environment, std::shared_ptr<CollisionDetection<dim>> collision,
                  MetricType metricType, NeighborType neighborType, PathModifierType modifierType, SamplerType samplerType,
                  SamplingType samplingType, TrajectoryType trajectoryType, const double trajecotryStepSize = 1,
                  const unsigned int samplingAttempts = 10, const double samplingDist = 10);

    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection();
    std::shared_ptr<DistanceMetric<dim>> getMetric();
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> getNeighborFinder();
    std::shared_ptr<Graph<dim>> getGraph();
    std::shared_ptr<PathModifier<dim>> getPathModifier();
    std::shared_ptr<Sampler<dim>> getSampler();
    std::shared_ptr<Sampling<dim>> getSampling();
    std::shared_ptr<TrajectoryPlanner<dim>> getTrajectoryPlanner();

    PlannerOptions<dim> getPlannerOptions();
    PRMOptions<dim> getPRMOptions(const double rangeSize);
    RRTOptions<dim> getRRTOptions(const double stepSize);
    SRTOptions<dim> getSRTOptions(const unsigned int nbOfTrees);

  private:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> m_neighborFinder = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier;
    std::shared_ptr<Sampler<dim>> m_sampler = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Constructor of the class ModuleCreator
*  \author     Sascha Kaden
*  \param[in]  environment
*  \param[in]  collision detection
*  \param[in]  metric type
*  \param[in]  neighbor type
*  \param[in]  sampler type
*  \param[in]  sampling type
*  \param[in]  sampling attempts
*  \param[in]  sampling distance for some specific sampler
*  \date       2017-05-22
*/
template <unsigned int dim>
ModuleCreator<dim>::ModuleCreator(std::shared_ptr<Environment> environment, std::shared_ptr<CollisionDetection<dim>> collision,
                                  MetricType metricType, NeighborType neighborType, PathModifierType modifierType,
                                  SamplerType samplerType, SamplingType samplingType, TrajectoryType trajectoryType,
                                  const double trajecotryStepSize, const unsigned int samplingAttempts, const double samplingDist)
    : Identifier("ModuleCreator") {
    m_collision = collision;

    if (trajectoryType == TrajectoryType::RotateAtS)
        m_trajectory = std::shared_ptr<TrajectoryPlanner<dim>>(new RotateAtS<dim>(collision, environment, trajecotryStepSize));
    else
        m_trajectory = std::shared_ptr<TrajectoryPlanner<dim>>(new LinearTrajectory<dim>(collision, environment, trajecotryStepSize));

    if (metricType == MetricType::L1)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new L1Metric<dim>());
    else if (metricType == MetricType::L2)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new L2Metric<dim>());
    else if (metricType == MetricType::Inf)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new InfMetric<dim>());

    if (neighborType == NeighborType::BruteForce)
        m_neighborFinder = std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>>(
            new BruteForceNF<dim, std::shared_ptr<Node<dim>>>(m_metric));
    else if (neighborType == NeighborType::KDTree)
        m_neighborFinder = std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>>(
            new KDTree<dim, std::shared_ptr<Node<dim>>>(m_metric));

    m_graph = std::shared_ptr<Graph<dim>>(new Graph<dim>(0, m_neighborFinder));

    if (modifierType == PathModifierType::NodeCut)
        m_pathModifier =
            std::shared_ptr<NodeCutPathModifier<dim>>(new NodeCutPathModifier<dim>(environment, collision, m_trajectory));
    else
        std::shared_ptr<DummyPathModifier<dim>>(new DummyPathModifier<dim>(environment, collision, m_trajectory));

    if (samplerType == SamplerType::SamplerRandom)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerRandom<dim>(environment));
    else if (samplerType == SamplerType::SamplerNormalDist)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerNormalDist<dim>(environment));
    else if (samplerType == SamplerType::SamplerUniform)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerUniform<dim>(environment));
    else if (samplerType == SamplerType::SeedSampler)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SeedSampler<dim>(environment));

    if (samplingType == SamplingType::Straight) {
        m_sampling =
            std::shared_ptr<Sampling<dim>>(new StraightSampling<dim>(environment, collision, m_trajectory, m_sampler));
    } else if (samplingType == SamplingType::Bridge) {
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new BridgeSampling<dim>(environment, collision, m_trajectory, m_sampler, samplingAttempts, samplingDist));
    } else if (samplingType == SamplingType::Gaussian) {
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new GaussianSampling<dim>(environment, collision, m_trajectory, m_sampler, samplingAttempts, samplingDist));
    } else if (samplingType == SamplingType::GaussianDist) {
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new GaussianDistSampling<dim>(environment, collision, m_trajectory, m_sampler, samplingAttempts, samplingDist));
    } else if (samplingType == SamplingType::MedialAxis) {
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new MedialAxisSampling<dim>(environment, collision, m_trajectory, m_sampler, samplingAttempts, samplingDist));
    } else if (samplingType == SamplingType::NearObstacle) {
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new SamplingNearObstacle<dim>(environment, collision, m_trajectory, m_sampler, samplingAttempts));
    }
}

/*!
*  \brief      Return the pointer to the CollisionDetection instance.
*  \author     Sascha Kaden
*  \param[out] CollisionDetection
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<CollisionDetection<dim>> ModuleCreator<dim>::getCollisionDetection() {
    return m_collision;
}

/*!
*  \brief      Return the pointer to the DistanceMetric instance.
*  \author     Sascha Kaden
*  \param[out] DistanceMetric
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<DistanceMetric<dim>> ModuleCreator<dim>::getMetric() {
    return m_metric;
}

/*!
*  \brief      Return the pointer to the distance metric instance.
*  \author     Sascha Kaden
*  \param[out] distance metric
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> ModuleCreator<dim>::getNeighborFinder() {
    return m_neighborFinder;
}

/*!
*  \brief      Return the pointer to the Graph instance.
*  \author     Sascha Kaden
*  \param[out] Graph
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> ModuleCreator<dim>::getGraph() {
    return m_graph;
}

/*!
*  \brief      Return the pointer to the PathModifier instance.
*  \author     Sascha Kaden
*  \param[out] PathModifier
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<PathModifier<dim>> ModuleCreator<dim>::getPathModifier() {
    return m_pathModifier;
}

/*!
*  \brief      Return the pointer to the Sampler instance.
*  \author     Sascha Kaden
*  \param[out] Sampler
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Sampler<dim>> ModuleCreator<dim>::getSampler() {
    return m_sampler;
}

/*!
*  \brief      Return the pointer to the Sampling instance.
*  \author     Sascha Kaden
*  \param[out] Sampling
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Sampling<dim>> ModuleCreator<dim>::getSampling() {
    return m_sampling;
}

/*!
*  \brief      Return the pointer to the TrajectoryPlanner instance.
*  \author     Sascha Kaden
*  \param[out] TrajecotryPlanner
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<TrajectoryPlanner<dim>> ModuleCreator<dim>::getTrajectoryPlanner() {
    return m_trajectory;
}

/*!
*  \brief      Generate PlannerOptions and return them.
*  \author     Sascha Kaden
*  \param[out] PlannerOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
PlannerOptions<dim> ModuleCreator<dim>::getPlannerOptions() {
    return PlannerOptions<dim>(m_collision, m_metric, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate PRMOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  range size
*  \param[out] PRMOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
PRMOptions<dim> ModuleCreator<dim>::getPRMOptions(const double rangeSize) {
    return PRMOptions<dim>(rangeSize, m_collision, m_metric, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate RRTOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  step size
*  \param[out] RRTOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
RRTOptions<dim> ModuleCreator<dim>::getRRTOptions(const double stepSize) {
    return RRTOptions<dim>(stepSize, m_collision, m_metric, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate SRTOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  number of trees
*  \param[out] SRTOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
SRTOptions<dim> ModuleCreator<dim>::getSRTOptions(const unsigned int nbOfTrees) {
    return SRTOptions<dim>(nbOfTrees, m_collision, m_metric, m_pathModifier, m_sampling, m_trajectory);
}

} /* namespace ippp */

#endif    // MODULECREATOR_HPP
