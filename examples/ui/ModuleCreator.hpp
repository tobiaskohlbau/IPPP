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

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>

namespace ippp {

enum class MetricType { L1, L2, Inf, L1Weighted, L2Weighted, InfWeighted };

enum class EvaluatorType { SingleIteration, Query };

enum class NeighborType { KDTree, BruteForce };

enum class PathModifierType { Dummy, NodeCut };

enum class SamplerType { SamplerRandom, SamplerNormalDist, SamplerUniform, SeedSampler };

enum class SamplingType { Bridge, Gaussian, GaussianDist, Straight, MedialAxis, NearObstacle };

enum class TrajectoryType { Linear, RotateAtS };

/*!
* \brief   Class ModuleCreator generates all defined modules for the path planner and creates the graph for the planner too. By a
* method it returns the options for the planner.
* \author  Sascha Kaden
* \date    2017-05-22
*/
template <unsigned int dim>
class ModuleCreator : public Identifier {
  public:
    ModuleCreator();

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

    void setEnvironment(const std::shared_ptr<Environment> &environment);
    void setCollision(const std::shared_ptr<CollisionDetection<dim>> &collision);
    void setMetricType(const MetricType type);
    void setMetricWeightVec(const Vector<dim> vector);
    void setEvaluatorType(const EvaluatorType type);
    void setQueryEvaluatorDist(const double queryEvaluatorDist);
    void setGraphSortCount(const size_t count);
    void setNeighborFinderType(const NeighborType type);
    void setPathModifierType(const PathModifierType type);
    void setSamplerType(const SamplerType type);
    void setSamplingProperties(const size_t samplingAttempts = 10, const double samplingDist = 10,
                               const double medialAxisDirs = 15);
    void setSamplingType(const SamplingType type);
    void setTrajectoryType(const TrajectoryType type);
    void setTrajectoryStepSize(const double stepSize);

  private:
    void initializeModules();

    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<Evaluator<dim>> m_evaluator = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> m_neighborFinder = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier;
    std::shared_ptr<Sampler<dim>> m_sampler = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;

    MetricType m_metricType = MetricType::L2;
    Vector<dim> m_metricWeight;
    EvaluatorType m_evaluatorType = EvaluatorType::SingleIteration;
    double m_queryEvaluatorDist = 10;
    size_t m_graphSortCount = 2000;
    NeighborType m_neighborType = NeighborType::KDTree;
    PathModifierType m_pathModifierType = PathModifierType::NodeCut;
    SamplerType m_samplerType = SamplerType::SamplerRandom;
    SamplingType m_samplingType = SamplingType::Straight;
    unsigned int m_samplingAttempts = 10;
    double m_samplingDist = 10;
    unsigned int m_medialAxisDirs = 15;
    TrajectoryType m_trajectoryType = TrajectoryType::Linear;
    double m_trajectoryStepSize = 1;
};

/*!
*  \brief      Constructor of the class ModuleCreator
*  \author     Sascha Kaden
*  \date       2017-10-03
*/
template <unsigned int dim>
ModuleCreator<dim>::ModuleCreator() : Identifier("ModuleCreator") {
}

/*!
*  \brief      Initialize all modules
*  \author     Sascha Kaden
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::initializeModules() {
    if (!m_collision || !m_environment) {
        Logging::error("CollisionDetection or Environment not set", this);
        return;
    }

    if (m_trajectoryType == TrajectoryType::RotateAtS)
        m_trajectory =
            std::shared_ptr<TrajectoryPlanner<dim>>(new RotateAtS<dim>(m_collision, m_environment, m_trajectoryStepSize));
    else
        m_trajectory =
            std::shared_ptr<TrajectoryPlanner<dim>>(new LinearTrajectory<dim>(m_collision, m_environment, m_trajectoryStepSize));

    if (m_metricType == MetricType::L1)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new L1Metric<dim>());
    else if (m_metricType == MetricType::Inf)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new InfMetric<dim>());
    else if (m_metricType == MetricType::L1Weighted)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new WeightedL1Metric<dim>(m_metricWeight));
    else if (m_metricType == MetricType::L2Weighted)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new WeightedL2Metric<dim>(m_metricWeight));
    else if (m_metricType == MetricType::InfWeighted)
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new WeightedInfMetric<dim>(m_metricWeight));
    else
        m_metric = std::shared_ptr<DistanceMetric<dim>>(new L2Metric<dim>());

    if (m_neighborType == NeighborType::BruteForce)
        m_neighborFinder = std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>>(
            new BruteForceNF<dim, std::shared_ptr<Node<dim>>>(m_metric));
    else
        m_neighborFinder = std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>>(
            new KDTree<dim, std::shared_ptr<Node<dim>>>(m_metric));

    m_graph = std::shared_ptr<Graph<dim>>(new Graph<dim>(m_graphSortCount, m_neighborFinder));

    if (m_evaluatorType == EvaluatorType::SingleIteration)
        m_evaluator = std::shared_ptr<Evaluator<dim>>(new SingleIterationEvaluator<dim>(m_environment));
    else
        m_evaluator =
            std::shared_ptr<Evaluator<dim>>(new QueryEvaluator<dim>(m_environment, m_metric, m_graph, m_queryEvaluatorDist));

    if (m_pathModifierType == PathModifierType::NodeCut)
        m_pathModifier =
            std::shared_ptr<PathModifier<dim>>(new NodeCutPathModifier<dim>(m_environment, m_collision, m_trajectory));
    else
        m_pathModifier = std::shared_ptr<PathModifier<dim>>(new DummyPathModifier<dim>(m_environment, m_collision, m_trajectory));

    if (m_samplerType == SamplerType::SamplerNormalDist)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerNormalDist<dim>(m_environment));
    else if (m_samplerType == SamplerType::SamplerUniform)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerUniform<dim>(m_environment));
    else if (m_samplerType == SamplerType::SeedSampler)
        m_sampler = std::shared_ptr<Sampler<dim>>(new SeedSampler<dim>(m_environment));
    else
        m_sampler = std::shared_ptr<Sampler<dim>>(new SamplerRandom<dim>(m_environment));

    if (m_samplingType == SamplingType::Bridge)
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new BridgeSampling<dim>(m_environment, m_collision, m_trajectory, m_sampler, m_samplingAttempts, m_samplingDist));
    else if (m_samplingType == SamplingType::Gaussian)
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new GaussianSampling<dim>(m_environment, m_collision, m_trajectory, m_sampler, m_samplingAttempts, m_samplingDist));
    else if (m_samplingType == SamplingType::GaussianDist)
        m_sampling = std::shared_ptr<Sampling<dim>>(new GaussianDistSampling<dim>(m_environment, m_collision, m_trajectory,
                                                                                  m_sampler, m_samplingAttempts, m_samplingDist));
    else if (m_samplingType == SamplingType::MedialAxis)
        m_sampling = std::shared_ptr<Sampling<dim>>(new MedialAxisSampling<dim>(
            m_environment, m_collision, m_trajectory, m_sampler, m_samplingAttempts, m_medialAxisDirs));
    else if (m_samplingType == SamplingType::NearObstacle)
        m_sampling = std::shared_ptr<Sampling<dim>>(
            new SamplingNearObstacle<dim>(m_environment, m_collision, m_trajectory, m_sampler, m_samplingAttempts));
    else
        m_sampling =
            std::shared_ptr<Sampling<dim>>(new StraightSampling<dim>(m_environment, m_collision, m_trajectory, m_sampler));
}

/*!
*  \brief      Sets the Environment
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setEnvironment(const std::shared_ptr<Environment> &environment) {
    m_environment = environment;
}

/*!
*  \brief      Sets the CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  CollisionDetection
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setCollision(const std::shared_ptr<CollisionDetection<dim>> &collision) {
    m_collision = collision;
}

/*!
*  \brief      Sets the MetricType
*  \author     Sascha Kaden
*  \param[in]  MetricType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setMetricType(const MetricType type) {
    m_metricType = type;
}

/*!
*  \brief      Sets the weight vector for the DistanceMetric
*  \author     Sascha Kaden
*  \param[in]  weight vector
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setMetricWeightVec(const Vector<dim> vector) {
    m_metricWeight = vector;
}

/*!
*  \brief      Sets the EvaluatorType
*  \author     Sascha Kaden
*  \param[in]  EvaluatorType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setEvaluatorType(const EvaluatorType type) {
    m_evaluatorType = type;
}

/*!
*  \brief      Sets the queryEvaluatorDist
*  \author     Sascha Kaden
*  \param[in]  queryEvaluatorDist
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setQueryEvaluatorDist(const double queryEvaluatorDist) {
    m_queryEvaluatorDist = queryEvaluatorDist;
}

/*!
*  \brief      Sets the Graph sort count
*  \author     Sascha Kaden
*  \param[in]  count
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setGraphSortCount(const size_t count) {
    m_graphSortCount = count;
}

/*!
*  \brief      Sets the NeighborType
*  \author     Sascha Kaden
*  \param[in]  NeighborType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setNeighborFinderType(const NeighborType type) {
    m_neighborType = type;
}

/*!
*  \brief      Sets the PathModifierType
*  \author     Sascha Kaden
*  \param[in]  PathModifierType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setPathModifierType(const PathModifierType type) {
    m_pathModifierType = type;
}

/*!
*  \brief      Sets the SamplerType
*  \author     Sascha Kaden
*  \param[in]  SamplerType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setSamplerType(const SamplerType type) {
    m_samplerType = type;
}

/*!
*  \brief      Sets the SamplingType
*  \author     Sascha Kaden
*  \param[in]  SamplingType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setSamplingType(const SamplingType type) {
    m_samplingType = type;
}

/*!
*  \brief      Sets the Sampling properties
*  \author     Sascha Kaden
*  \param[in]  sampling attempts
*  \param[in]  sampling distance
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setSamplingProperties(const size_t samplingAttempts, const double samplingDist,
                                               const double medialAxisDirs) {
    m_samplingAttempts = samplingAttempts;
    m_samplingDist = samplingDist;
    m_medialAxisDirs = medialAxisDirs;
}

/*!
*  \brief      Sets the TrajectoryType
*  \author     Sascha Kaden
*  \param[in]  TrajectoryType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setTrajectoryType(const TrajectoryType type) {
    m_trajectoryType = type;
}

/*!
*  \brief      Sets the trajectory step size
*  \author     Sascha Kaden
*  \param[in]  step size
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleCreator<dim>::setTrajectoryStepSize(const double stepSize) {
    m_trajectoryStepSize = stepSize;
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
    return PlannerOptions<dim>(m_collision, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
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
    initializeModules();
    return PRMOptions<dim>(rangeSize, m_collision, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
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
    initializeModules();
    return RRTOptions<dim>(stepSize, m_collision, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
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
    initializeModules();
    return SRTOptions<dim>(nbOfTrees, m_collision, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

} /* namespace ippp */

#endif    // MODULECREATOR_HPP
