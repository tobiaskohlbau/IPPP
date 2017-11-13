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

#ifndef MODULECONFIGURATOR_HPP
#define MODULECONFIGURATOR_HPP

#include <fstream>
#include <type_traits>
#include <vector>

#include <ippp/Core.h>
#include <ippp/Planner.h>
#include <ippp/ui/Configurator.h>

namespace ippp {

enum class CollisionType { Dim2, Dim2Triangle, AlwaysValid, PQP, FCL, AABB, Sphere };

enum class MetricType { L1, L2, Inf, L1Weighted, L2Weighted, InfWeighted };

enum class EvaluatorType { SingleIteration, Query, Time, QueryOrTime };

enum class NeighborType { KDTree, BruteForce };

enum class PathModifierType { Dummy, NodeCut };

enum class SamplerType { SamplerRandom, SamplerNormalDist, SamplerUniform, SeedSampler, GridSampler };

enum class SamplingType { Bridge, Gaussian, GaussianDist, Straight, MedialAxis, NearObstacle };

enum class TrajectoryType { Linear, RotateAtS };

/*!
* \brief   Class ModuleConfigurator generates all defined modules for the path planner and creates the graph for the planner too.
* By a method it returns the options for the planner.
* \author  Sascha Kaden
* \date    2017-05-22
*/
template <unsigned int dim>
class ModuleConfigurator : public Configurator {
  public:
    ModuleConfigurator();
    bool saveConfig(const std::string &filePath);
    bool loadConfig(const std::string &filePath);

    std::shared_ptr<Environment> getEnvironment();
    std::shared_ptr<CollisionDetection<dim>> getCollisionDetection();
    std::shared_ptr<DistanceMetric<dim>> getDistanceMetric();
    std::shared_ptr<Evaluator<dim>> getEvaluator();
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
    void setCollisionType(const CollisionType type);
    void setMetricType(const MetricType type);
    void setMetricWeightVec(const Vector<dim> vector);
    void setEvaluatorType(const EvaluatorType type);
    void setEvaluatorProperties(const double queryEvaluatorDist, const size_t duration);
    void setGraphSortCount(const size_t count);
    void setNeighborFinderType(const NeighborType type);
    void setPathModifierType(const PathModifierType type);
    void setSamplerType(const SamplerType type);
    void setSamplingProperties(const size_t samplingAttempts = 10, const double samplingDist = 10,
                               const size_t medialAxisDirs = 15);
    void setSamplingType(const SamplingType type);
    void setTrajectoryType(const TrajectoryType type);
    void setTrajectoryProperties(const double posRes, const double oriRes);

  protected:
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

    CollisionType m_collisionType = CollisionType::PQP;
    MetricType m_metricType = MetricType::L2;
    Vector<dim> m_metricWeight;
    EvaluatorType m_evaluatorType = EvaluatorType::SingleIteration;
    double m_queryEvaluatorDist = 10;
    size_t m_evaluatorDuration = 10;
    size_t m_graphSortCount = 2000;
    NeighborType m_neighborType = NeighborType::KDTree;
    PathModifierType m_pathModifierType = PathModifierType::NodeCut;
    SamplerType m_samplerType = SamplerType::SamplerRandom;
    SamplingType m_samplingType = SamplingType::Straight;
    size_t m_samplingAttempts = 10;
    double m_samplingDist = 10;
    size_t m_medialAxisDirs = 15;
    TrajectoryType m_trajectoryType = TrajectoryType::Linear;
    double m_posRes = 1;
    double m_oriRes = 0.1;

    bool m_parameterModified = false;
};

/*!
*  \brief      Constructor of the class ModuleConfigurator
*  \author     Sascha Kaden
*  \date       2017-10-03
*/
template <unsigned int dim>
ModuleConfigurator<dim>::ModuleConfigurator() : Configurator("ModuleConfigurator") {
}

/*!
*  \brief      Initialize all modules
*  \author     Sascha Kaden
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::initializeModules() {
    if (!m_environment) {
        Logging::error("Environment not set!", this);
        return;
    }

    if (!m_parameterModified)
        return;
    m_parameterModified = false;

    switch (m_collisionType) {
        case ippp::CollisionType::Dim2:
            m_collision = std::make_shared<CollisionDetection2D<dim>>(m_environment);
            break;
        case ippp::CollisionType::Dim2Triangle:
            m_collision = std::make_shared<CollisionDetectionTriangleRobot<dim>>(m_environment);
            break;
        case ippp::CollisionType::AlwaysValid:
            m_collision = std::make_shared<CollisionDetectionAlwaysValid<dim>>(m_environment);
            break;
        case ippp::CollisionType::FCL:
            m_collision = std::make_shared<CollisionDetectionFcl<dim>>(m_environment);
            break;
        case ippp::CollisionType::AABB:
            m_collision = std::make_shared<CollisionDetectionAABB<dim>>(m_environment);
            break;
        case ippp::CollisionType::Sphere:
            m_collision = std::make_shared<CollisionDetectionSphere<dim>>(m_environment);
            break;
        default:
            m_collision = std::make_shared<CollisionDetectionPqp<dim>>(m_environment);
            break;
    }

    switch (m_trajectoryType) {
        case ippp::TrajectoryType::Linear:
            m_trajectory = std::make_shared<LinearTrajectory<dim>>(m_collision, m_environment, m_posRes, m_oriRes);
            break;
        case ippp::TrajectoryType::RotateAtS:
            m_trajectory = std::make_shared<RotateAtS<dim>>(m_collision, m_environment, m_posRes, m_oriRes);
            break;
        default:
            m_trajectory = std::make_shared<LinearTrajectory<dim>>(m_collision, m_environment, m_posRes, m_oriRes);
            break;
    }

    switch (m_metricType) {
        case ippp::MetricType::L1:
            m_metric = std::make_shared<L1Metric<dim>>();
            break;
        case ippp::MetricType::L2:
            m_metric = std::make_shared<L2Metric<dim>>();
            break;
        case ippp::MetricType::Inf:
            m_metric = std::make_shared<InfMetric<dim>>();
            break;
        case ippp::MetricType::L1Weighted:
            m_metric = std::make_shared<WeightedL1Metric<dim>>(m_metricWeight);
            break;
        case ippp::MetricType::L2Weighted:
            m_metric = std::make_shared<WeightedL2Metric<dim>>(m_metricWeight);
            break;
        case ippp::MetricType::InfWeighted:
            m_metric = std::make_shared<WeightedInfMetric<dim>>(m_metricWeight);
            break;
        default:
            m_metric = std::make_shared<L2Metric<dim>>();
            break;
    }

    switch (m_neighborType) {
        case ippp::NeighborType::KDTree:
            m_neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
        case ippp::NeighborType::BruteForce:
            m_neighborFinder = std::make_shared<BruteForceNF<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
        default:
            m_neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
    }

    m_graph = std::make_shared<Graph<dim>>(m_graphSortCount, m_neighborFinder);

    switch (m_evaluatorType) {
        case ippp::EvaluatorType::SingleIteration:
            m_evaluator = std::make_shared<SingleIterationEvaluator<dim>>();
            break;
        case ippp::EvaluatorType::Query:
            m_evaluator = std::make_shared<QueryEvaluator<dim>>(m_metric, m_graph, m_queryEvaluatorDist);
            break;
        case ippp::EvaluatorType::Time:
            m_evaluator = std::make_shared<TimeEvaluator<dim>>(m_evaluatorDuration);
            break;
        case ippp::EvaluatorType::QueryOrTime:
            std::vector<std::shared_ptr<Evaluator<dim>>> evaluators;
            evaluators.push_back(
                std::shared_ptr<Evaluator<dim>>(new QueryEvaluator<dim>(m_metric, m_graph, m_queryEvaluatorDist)));
            evaluators.push_back(std::shared_ptr<Evaluator<dim>>(new TimeEvaluator<dim>(m_evaluatorDuration)));
            m_evaluator = std::shared_ptr<Evaluator<dim>>(new ComposeEvaluator<dim>(evaluators, ComposeType::OR));
            break;
    }

    switch (m_pathModifierType) {
        case ippp::PathModifierType::Dummy:
            m_pathModifier = std::make_shared<DummyPathModifier<dim>>(m_environment, m_collision, m_trajectory);
            break;
        case ippp::PathModifierType::NodeCut:
            m_pathModifier = std::make_shared<NodeCutPathModifier<dim>>(m_environment, m_collision, m_trajectory);
            break;
        default:
            m_pathModifier = std::make_shared<NodeCutPathModifier<dim>>(m_environment, m_collision, m_trajectory);
            break;
    }

    switch (m_samplerType) {
        case ippp::SamplerType::SamplerRandom:
            m_sampler = std::make_shared<SamplerRandom<dim>>(m_environment);
            break;
        case ippp::SamplerType::SamplerNormalDist:
            m_sampler = std::make_shared<SamplerNormalDist<dim>>(m_environment);
            break;
        case ippp::SamplerType::SamplerUniform:
            m_sampler = std::make_shared<SamplerUniform<dim>>(m_environment);
            break;
        case ippp::SamplerType::SeedSampler:
            m_sampler = std::make_shared<SeedSampler<dim>>(m_environment);
            break;
        case ippp::SamplerType::GridSampler:
            m_sampler = std::make_shared<GridSampler<dim>>(m_environment);
        default:
            m_sampler = std::make_shared<SamplerUniform<dim>>(m_environment);
            break;
    }

    switch (m_samplingType) {
        case ippp::SamplingType::Bridge:
            m_sampling = std::make_shared<BridgeSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler,
                                                               m_samplingAttempts, m_samplingDist);
            break;
        case ippp::SamplingType::Gaussian:
            m_sampling = std::make_shared<GaussianSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler,
                                                                 m_samplingAttempts, m_samplingDist);
            break;
        case ippp::SamplingType::GaussianDist:
            m_sampling = std::make_shared<GaussianDistSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler,
                                                                     m_samplingAttempts, m_samplingDist);
            break;
        case ippp::SamplingType::Straight:
            m_sampling = std::make_shared<StraightSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler);
            break;
        case ippp::SamplingType::MedialAxis:
            m_sampling = std::make_shared<MedialAxisSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler,
                                                                   m_samplingAttempts, m_medialAxisDirs);
            break;
        case ippp::SamplingType::NearObstacle:
            m_sampling = std::make_shared<SamplingNearObstacle<dim>>(m_environment, m_collision, m_trajectory, m_sampler,
                                                                     m_samplingAttempts);
            break;
        default:
            m_sampling = std::make_shared<StraightSampling<dim>>(m_environment, m_collision, m_trajectory, m_sampler);
            break;
    }
}

template <unsigned int dim>
bool ModuleConfigurator<dim>::saveConfig(const std::string &filePath) {
    // types
    nlohmann::json json;
    json["CollisionType"] = static_cast<int>(m_collisionType);
    json["MetricType"] = static_cast<int>(m_metricType);
    json["MetricWeight"] = vectorToString<dim>(m_metricWeight);
    json["EvaluatorType"] = static_cast<int>(m_evaluatorType);
    json["QueryEvaluatorDist"] = m_queryEvaluatorDist;
    json["EvaluatorDuration"] = m_evaluatorDuration;
    json["GraphSortCount"] = m_graphSortCount;
    json["NeighborType"] = static_cast<int>(m_neighborType);
    json["PathModifierType"] = static_cast<int>(m_pathModifierType);
    json["SamplerType"] = static_cast<int>(m_samplerType);
    json["SamplingType"] = static_cast<int>(m_samplingType);
    json["SamplingAttempts"] = m_samplingAttempts;
    json["SamplingDist"] = m_samplingDist;
    json["MedialAxisDirs"] = m_medialAxisDirs;
    json["TrajectoryType"] = static_cast<int>(m_trajectoryType);
    json["PosRes"] = m_posRes;
    json["OriRes"] = m_oriRes;

    return saveJson(filePath, json);
}

template <unsigned int dim>
bool ModuleConfigurator<dim>::loadConfig(const std::string &filePath) {
    nlohmann::json json = loadJson(filePath);
    if (json.empty())
        return false;

    m_collisionType = static_cast<CollisionType>(json["CollisionType"].get<int>());
    m_metricType = static_cast<MetricType>(json["MetricType"].get<int>());
    m_metricWeight = stringToVector<dim>(json["MetricWeight"].get<std::string>());
    m_evaluatorType = static_cast<EvaluatorType>(json["EvaluatorType"].get<int>());
    m_queryEvaluatorDist = json["QueryEvaluatorDist"].get<double>();
    m_evaluatorDuration = json["EvaluatorDuration"].get<size_t>();
    m_graphSortCount = json["GraphSortCount"].get<size_t>();
    m_neighborType = static_cast<NeighborType>(json["NeighborType"].get<int>());
    m_pathModifierType = static_cast<PathModifierType>(json["PathModifierType"].get<int>());
    m_samplerType = static_cast<SamplerType>(json["SamplerType"].get<int>());
    m_samplingType = static_cast<SamplingType>(json["SamplingType"].get<int>());
    m_samplingAttempts = json["SamplingAttempts"].get<size_t>();
    m_samplingDist = json["SamplingDist"].get<double>();
    m_medialAxisDirs = json["MedialAxisDirs"].get<size_t>();
    m_trajectoryType = static_cast<TrajectoryType>(json["TrajectoryType"].get<int>());
    m_posRes = json["PosRes"].get<double>();
    m_posRes = json["OriRes"].get<double>();
    initializeModules();

    return true;
}

/*!
*  \brief      Sets the Environment
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setEnvironment(const std::shared_ptr<Environment> &environment) {
    m_environment = environment;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the CollisionType
*  \author     Sascha Kaden
*  \param[in]  CollisionType
*  \date       2017-10-06
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setCollisionType(const CollisionType type) {
    m_collisionType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the MetricType
*  \author     Sascha Kaden
*  \param[in]  MetricType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setMetricType(const MetricType type) {
    m_metricType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the weight vector for the DistanceMetric
*  \author     Sascha Kaden
*  \param[in]  weight vector
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setMetricWeightVec(const Vector<dim> vector) {
    m_metricWeight = vector;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the EvaluatorType
*  \author     Sascha Kaden
*  \param[in]  EvaluatorType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setEvaluatorType(const EvaluatorType type) {
    m_evaluatorType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the queryEvaluatorDist
*  \author     Sascha Kaden
*  \param[in]  query evaluator distance
*  \param[in]  duration time in seconds
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setEvaluatorProperties(const double queryEvaluatorDist, const size_t duration) {
    m_queryEvaluatorDist = queryEvaluatorDist;
    m_evaluatorDuration = duration;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the Graph sort count
*  \author     Sascha Kaden
*  \param[in]  count
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setGraphSortCount(const size_t count) {
    m_graphSortCount = count;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the NeighborType
*  \author     Sascha Kaden
*  \param[in]  NeighborType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setNeighborFinderType(const NeighborType type) {
    m_neighborType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the PathModifierType
*  \author     Sascha Kaden
*  \param[in]  PathModifierType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setPathModifierType(const PathModifierType type) {
    m_pathModifierType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the SamplerType
*  \author     Sascha Kaden
*  \param[in]  SamplerType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setSamplerType(const SamplerType type) {
    m_samplerType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the SamplingType
*  \author     Sascha Kaden
*  \param[in]  SamplingType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setSamplingType(const SamplingType type) {
    m_samplingType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the Sampling properties
*  \author     Sascha Kaden
*  \param[in]  sampling attempts
*  \param[in]  sampling distance
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setSamplingProperties(const size_t samplingAttempts, const double samplingDist,
                                                    const size_t medialAxisDirs) {
    m_samplingAttempts = samplingAttempts;
    m_samplingDist = samplingDist;
    m_medialAxisDirs = medialAxisDirs;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the TrajectoryType
*  \author     Sascha Kaden
*  \param[in]  TrajectoryType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setTrajectoryType(const TrajectoryType type) {
    m_trajectoryType = type;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the trajectory step size
*  \author     Sascha Kaden
*  \param[in]  step size
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setTrajectoryProperties(const double posRes, const double oriRes) {
    if (posRes <= 0) {
        m_posRes = 1;
        Logging::warning("Position resolution has to be larger than 0, it was set to 1!", this);
    } else {
        m_posRes = posRes;
    }
    if (oriRes <= 0) {
        m_oriRes = 0.1;
        Logging::warning("Orientation resolution has to be larger than 0, it was set to 0.1!", this);
    } else {
        m_oriRes = oriRes;
    }
    m_parameterModified = true;
}

/*!
*  \brief      Return the pointer to the Environment instance.
*  \author     Sascha Kaden
*  \param[out] Evaluator
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Environment> ModuleConfigurator<dim>::getEnvironment() {
    return m_environment;
}

/*!
*  \brief      Return the pointer to the CollisionDetection instance.
*  \author     Sascha Kaden
*  \param[out] CollisionDetection
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<CollisionDetection<dim>> ModuleConfigurator<dim>::getCollisionDetection() {
    initializeModules();
    return m_collision;
}

/*!
*  \brief      Return the pointer to the DistanceMetric instance.
*  \author     Sascha Kaden
*  \param[out] DistanceMetric
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<DistanceMetric<dim>> ModuleConfigurator<dim>::getDistanceMetric() {
    initializeModules();
    return m_metric;
}

/*!
*  \brief      Return the pointer to the Evaluator instance.
*  \author     Sascha Kaden
*  \param[out] Evaluator
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Evaluator<dim>> ModuleConfigurator<dim>::getEvaluator() {
    initializeModules();
    return m_evaluator;
}

/*!
*  \brief      Return the pointer to the distance metric instance.
*  \author     Sascha Kaden
*  \param[out] distance metric
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> ModuleConfigurator<dim>::getNeighborFinder() {
    initializeModules();
    return m_neighborFinder;
}

/*!
*  \brief      Return the pointer to the Graph instance.
*  \author     Sascha Kaden
*  \param[out] Graph
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> ModuleConfigurator<dim>::getGraph() {
    initializeModules();
    return m_graph;
}

/*!
*  \brief      Return the pointer to the PathModifier instance.
*  \author     Sascha Kaden
*  \param[out] PathModifier
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<PathModifier<dim>> ModuleConfigurator<dim>::getPathModifier() {
    initializeModules();
    return m_pathModifier;
}

/*!
*  \brief      Return the pointer to the Sampler instance.
*  \author     Sascha Kaden
*  \param[out] Sampler
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Sampler<dim>> ModuleConfigurator<dim>::getSampler() {
    initializeModules();
    return m_sampler;
}

/*!
*  \brief      Return the pointer to the Sampling instance.
*  \author     Sascha Kaden
*  \param[out] Sampling
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Sampling<dim>> ModuleConfigurator<dim>::getSampling() {
    initializeModules();
    return m_sampling;
}

/*!
*  \brief      Return the pointer to the TrajectoryPlanner instance.
*  \author     Sascha Kaden
*  \param[out] TrajecotryPlanner
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<TrajectoryPlanner<dim>> ModuleConfigurator<dim>::getTrajectoryPlanner() {
    initializeModules();
    return m_trajectory;
}

/*!
*  \brief      Generate PlannerOptions and return them.
*  \author     Sascha Kaden
*  \param[out] PlannerOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
PlannerOptions<dim> ModuleConfigurator<dim>::getPlannerOptions() {
    initializeModules();
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
PRMOptions<dim> ModuleConfigurator<dim>::getPRMOptions(const double rangeSize) {
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
RRTOptions<dim> ModuleConfigurator<dim>::getRRTOptions(const double stepSize) {
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
SRTOptions<dim> ModuleConfigurator<dim>::getSRTOptions(const unsigned int nbOfTrees) {
    initializeModules();
    return SRTOptions<dim>(nbOfTrees, m_collision, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

} /* namespace ippp */

#endif    // MODULECONFIGURATOR_HPP
