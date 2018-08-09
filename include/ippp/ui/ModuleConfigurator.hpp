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

#ifndef MODULECONFIGURATOR_HPP
#define MODULECONFIGURATOR_HPP

#include <string>
#include <type_traits>
#include <vector>

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/ui/Configurator.h>

namespace ippp {

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
    void resetModules();

    std::shared_ptr<Environment> getEnvironment();
    std::shared_ptr<DistanceMetric<dim>> getDistanceMetric();
    std::shared_ptr<Evaluator<dim>> getEvaluator();
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> getNeighborFinder();
    std::shared_ptr<Graph<dim>> getGraph();
    std::shared_ptr<Graph<dim>> getGraphB();
    std::shared_ptr<PathModifier<dim>> getPathModifier();
    std::shared_ptr<Sampler<dim>> getSampler();
    std::shared_ptr<Sampling<dim>> getSampling();
    std::shared_ptr<TrajectoryPlanner<dim>> getTrajectoryPlanner();
    std::shared_ptr<ValidityChecker<dim>> getValidityChecker();

    MPOptions<dim> getPlannerOptions();
    PRMOptions<dim> getPRMOptions(double rangeSize);
    RRTOptions<dim> getRRTOptions(double stepSize);
    SRTOptions<dim> getSRTOptions(unsigned int nbOfTrees);

    void setEnvironment(const std::shared_ptr<Environment> &environment);
    void setMetricType(DistanceMetricType type);
    void setMetricWeightVec(const Vector<dim> vector);
    void setEvaluatorType(EvaluatorType type);
    void setEvaluatorProperties(double queryEvaluatorDist, size_t duration,
                                const std::pair<Vector6, Vector6> &C = std::make_pair(Vector6::Zero(), Vector6::Zero()));
    void setGraphSortCount(size_t count);
    void setNeighborFinderType(NeighborFinderType type);
    void setPathModifierType(PathModifierType type);
    void setSamplerType(SamplerType type);
    void setSamplerProperties(const std::string &seed, double gridResolution = 1);
    void setSamplingProperties(size_t samplingAttempts = 10, double samplingDist = 10, size_t medialAxisDirs = 15);
    void setSamplingType(SamplingType type);
    void setTrajectoryType(TrajectoryPlannerType type);
    void setTrajectoryProperties(double posRes, double oriRes);
    void setValidityCheckerType(ValidityCheckerType type);
    void setConstraintProperties(const std::pair<Vector6, Vector6> &C, const Transform &taskFrame,
                                 const double epsilon = IPPP_EPSILON);
    void setGoalPose(const Vector6 goalPose);

  protected:
    void initializeModules();

    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Environment> m_env = nullptr;
    std::shared_ptr<Evaluator<dim>> m_evaluator = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<Graph<dim>> m_graphB = nullptr;
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> m_neighborFinder = nullptr;
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> m_neighborFinderB = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier;
    std::shared_ptr<Sampler<dim>> m_sampler = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;

    DistanceMetricType m_metricType = DistanceMetricType::L2;
    Vector<dim> m_metricWeight = Vector<dim>::Zero();
    EvaluatorType m_evaluatorType = EvaluatorType::SingleIteration;
    double m_queryEvalDist = 10;
    size_t m_evaluatorDuration = 10;
    size_t m_graphSortCount = 3000;
    NeighborFinderType m_neighborType = NeighborFinderType::KDTree;
    PathModifierType m_pathModifierType = PathModifierType::NodeCut;
    SamplerType m_samplerType = SamplerType::UniformBiased;
    std::string m_samplerSeed = "";
    double m_samplerGridResolution = 1;
    SamplingType m_samplingType = SamplingType::Straight;
    size_t m_samplingAttempts = 10;
    double m_samplingDist = 10;
    size_t m_medialAxisDirs = 15;
    TrajectoryPlannerType m_trajectoryType = TrajectoryPlannerType::Linear;
    double m_posRes = 1;
    double m_oriRes = util::toRad(2.5);
    ValidityCheckerType m_validityType = ValidityCheckerType::FclSerial;
    // constraint
    double m_constraintEpsilon;
    std::pair<Vector6, Vector6> m_goalC;
    std::pair<Vector6, Vector6> m_taskFrameC;
    Transform m_taskFrame;
    Vector6 m_goalPose;

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
    if (!m_env) {
        Logging::error("Environment not set!", this);
        return;
    }

    if (!m_parameterModified)
        return;
    m_parameterModified = false;

    switch (m_validityType) {
        case ValidityCheckerType::Dim2:
            m_validityChecker = std::make_shared<CollisionDetection2D<dim>>(m_env);
            break;
        case ValidityCheckerType::FclMobile:
            m_validityChecker = std::make_shared<CollisionFclMobile<dim>>(m_env);
            break;
        case ValidityCheckerType::FclSerial:
            m_validityChecker = std::make_shared<CollisionFclSerial<dim>>(m_env);
            break;
        case ValidityCheckerType::AABB:
            m_validityChecker = std::make_shared<CollisionDetectionAABB<dim>>(m_env);
            break;
        case ValidityCheckerType::AlwaysValid:
            m_validityChecker = std::make_shared<AlwaysTrueValidity<dim>>(m_env);
            break;
        case ValidityCheckerType::BerensonConstraint:
            m_validityChecker = std::make_shared<BerensonConstraint<dim>>(m_env, m_taskFrame, m_taskFrameC);
            break;
        case ValidityCheckerType::FclSerialAndConstraint:
            m_validityChecker = std::make_shared<ComposeValidity<dim>>(
                m_env, std::vector<std::shared_ptr<ValidityChecker<dim>>>(
                           {std::make_shared<CollisionFclSerial<dim>>(m_env),
                            std::make_shared<BerensonConstraint<dim>>(m_env, m_taskFrame, m_taskFrameC)}),
                ComposeType::AND);
            break;
        default:
            m_validityChecker = std::make_shared<CollisionDetectionPqp<dim>>(m_env);
            break;
    }

    switch (m_trajectoryType) {
        case TrajectoryPlannerType::Linear:
            m_trajectory = std::make_shared<LinearTrajectory<dim>>(m_env, m_posRes, m_oriRes);
            break;
        case TrajectoryPlannerType::RotateAtS:
            m_trajectory = std::make_shared<RotateAtS<dim>>(m_env, m_posRes, m_oriRes);
            break;
        default:
            m_trajectory = std::make_shared<LinearTrajectory<dim>>(m_env, m_posRes, m_oriRes);
            break;
    }

    switch (m_metricType) {
        case DistanceMetricType::L1:
            m_metric = std::make_shared<L1Metric<dim>>();
            break;
        case DistanceMetricType::L2:
            m_metric = std::make_shared<L2Metric<dim>>();
            break;
        case DistanceMetricType::Inf:
            m_metric = std::make_shared<InfMetric<dim>>();
            break;
        case DistanceMetricType::L1Weighted:
            m_metric = std::make_shared<WeightedL1Metric<dim>>(m_metricWeight);
            break;
        case DistanceMetricType::L2Weighted:
            m_metric = std::make_shared<WeightedL2Metric<dim>>(m_metricWeight);
            break;
        case DistanceMetricType::InfWeighted:
            m_metric = std::make_shared<WeightedInfMetric<dim>>(m_metricWeight);
            break;
        default:
            m_metric = std::make_shared<L2Metric<dim>>();
            break;
    }

    switch (m_neighborType) {
        case NeighborFinderType::KDTree:
            m_neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            m_neighborFinderB = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
        case NeighborFinderType::BruteForce:
            m_neighborFinder = std::make_shared<BruteForceNF<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            m_neighborFinderB = std::make_shared<BruteForceNF<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
        default:
            m_neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            m_neighborFinderB = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(m_metric);
            break;
    }

    m_graph = std::make_shared<Graph<dim>>(m_graphSortCount, m_neighborFinder);

    switch (m_evaluatorType) {
        case EvaluatorType::SingleIteration:
            m_evaluator = std::make_shared<SingleIterationEvaluator<dim>>();
            break;
        case EvaluatorType::Time:
            m_evaluator = std::make_shared<TimeEvaluator<dim>>(m_evaluatorDuration);
            break;
        case EvaluatorType::TreeConfig:
            m_evaluator =
                std::make_shared<TreeConfigEvaluator<dim>>(m_metric, m_graph, m_trajectory, m_validityChecker, m_queryEvalDist);
            break;
        case EvaluatorType::TreeConnect:
            m_graphB = std::make_shared<Graph<dim>>(m_graphSortCount, m_neighborFinderB, "GraphB");
            m_evaluator =
                std::make_shared<TreeConnectEvaluator<dim>>(m_graph, m_graphB, m_trajectory, m_validityChecker, m_queryEvalDist);
            break;
        case EvaluatorType::TreePose:
            m_evaluator = std::make_shared<TreePoseEvaluator<dim>>(m_graph, m_env, m_goalC);
            break;
        case EvaluatorType::PRMConfig:
            m_evaluator = std::make_shared<PRMConfigEvaluator<dim>>(m_graph, m_trajectory, m_validityChecker, m_queryEvalDist);
            break;
        case EvaluatorType::PRMPose:
            m_evaluator = std::make_shared<PRMPoseEvaluator<dim>>(m_env, m_graph, m_trajectory, m_validityChecker, m_goalC,
                                                                  m_queryEvalDist);
            break;
        case EvaluatorType::TreeConfigOrTime:
            m_evaluator = std::make_shared<ComposeEvaluator<dim>>(
                std::vector<std::shared_ptr<Evaluator<dim>>>(
                    {std::make_shared<TreeConfigEvaluator<dim>>(m_metric, m_graph, m_trajectory, m_validityChecker,
                                                                m_queryEvalDist),
                     std::make_shared<TimeEvaluator<dim>>(m_evaluatorDuration)}),
                ComposeType::OR);
            break;
        case EvaluatorType::TreeConnectOrTime:
            m_graphB = std::make_shared<Graph<dim>>(m_graphSortCount, m_neighborFinderB, "GraphB");
            m_evaluator = std::make_shared<ComposeEvaluator<dim>>(
                std::vector<std::shared_ptr<Evaluator<dim>>>(
                    {std::make_shared<TreeConnectEvaluator<dim>>(m_graph, m_graphB, m_trajectory, m_validityChecker,
                                                                 m_queryEvalDist),
                     std::make_shared<TimeEvaluator<dim>>(m_evaluatorDuration)}),
                ComposeType::OR);
            break;

        case EvaluatorType::TreePoseOrTime:
            m_evaluator = std::make_shared<ComposeEvaluator<dim>>(
                std::vector<std::shared_ptr<Evaluator<dim>>>({std::make_shared<TreePoseEvaluator<dim>>(m_graph, m_env, m_goalC),
                                                              std::make_shared<TimeEvaluator<dim>>(m_evaluatorDuration)}),
                ComposeType::OR);
            break;
    }

    switch (m_pathModifierType) {
        case PathModifierType::Dummy:
            m_pathModifier = std::make_shared<DummyPathModifier<dim>>();
            break;
        case PathModifierType::NodeCut:
            m_pathModifier = std::make_shared<NodeCutPathModifier<dim>>(m_env, m_trajectory, m_validityChecker);
            break;
        default:
            m_pathModifier = std::make_shared<NodeCutPathModifier<dim>>(m_env, m_trajectory, m_validityChecker);
            break;
    }

    switch (m_samplerType) {
        case SamplerType::Random:
            m_sampler = std::make_shared<SamplerRandom<dim>>(m_env, m_samplerSeed);
            break;
        case SamplerType::NormalDist:
            m_sampler = std::make_shared<SamplerNormalDist<dim>>(m_env, m_samplerSeed);
            break;
        case SamplerType::Uniform:
            m_sampler = std::make_shared<SamplerUniform<dim>>(m_env, m_samplerSeed);
            break;
        case SamplerType::Grid:
            m_sampler = std::make_shared<GridSampler<dim>>(m_env, m_samplerGridResolution);
            break;
        case SamplerType::UniformBiased:
            m_sampler = std::make_shared<SamplerUniformBiased<dim>>(m_env, m_graph, m_samplerSeed);
            break;
        case SamplerType::InverseJacobi:
            m_sampler = std::make_shared<SamplerInverseJacobi<dim>>(m_env, m_goalPose, m_samplerSeed);
            break;
        default:
            m_sampler = std::make_shared<SamplerUniform<dim>>(m_env, m_samplerSeed);
            break;
    }

    switch (m_samplingType) {
        case SamplingType::Bridge:
            m_sampling =
                std::make_shared<BridgeSampling<dim>>(m_env, m_sampler, m_validityChecker, m_samplingAttempts, m_samplingDist);
            break;
        case SamplingType::Gaussian:
            m_sampling =
                std::make_shared<GaussianSampling<dim>>(m_env, m_sampler, m_validityChecker, m_samplingAttempts, m_samplingDist);
            break;
        case SamplingType::GaussianDist:
            m_sampling = std::make_shared<GaussianDistSampling<dim>>(m_env, m_graph, m_sampler, m_validityChecker,
                                                                     m_samplingAttempts, m_samplingDist);
            break;
        case SamplingType::Straight:
            m_sampling = std::make_shared<StraightSampling<dim>>(m_env, m_sampler, m_validityChecker);
            break;
        case SamplingType::MedialAxis:
            m_sampling = std::make_shared<MedialAxisSampling<dim>>(m_env, m_sampler, m_validityChecker, m_samplingAttempts,
                                                                   m_medialAxisDirs);
            break;
        case SamplingType::NearObstacle:
            m_sampling = std::make_shared<SamplingNearObstacle<dim>>(m_env, m_sampler, m_validityChecker, m_samplingAttempts,
                                                                     m_trajectory);
            break;
        case SamplingType::TS:
            m_sampling = std::make_shared<TangentSpaceSampling<dim>>(
                m_env, m_graph, m_sampler, m_validityChecker, m_samplingAttempts, m_samplingDist, m_taskFrameC, m_taskFrame);
            break;
        case SamplingType::RGD:
            m_sampling = std::make_shared<RGDSampling<dim>>(m_env, m_graph, m_sampler, m_validityChecker, m_samplingAttempts);
            break;
        case SamplingType::FOR:
            if (m_validityType == ValidityCheckerType::FclSerialAndConstraint) {
                auto checker = std::dynamic_pointer_cast<ComposeValidity<dim>>(m_validityChecker);
                m_sampling = std::make_shared<FirstOrderRetractionSampling<dim>>(
                    m_env, m_graph, m_sampler, std::dynamic_pointer_cast<Constraint<dim>>(checker->getChecker(1)),
                    m_validityChecker, m_samplingAttempts, m_samplingDist, m_taskFrame);
            } else if (m_validityType == ValidityCheckerType::BerensonConstraint) {
                m_sampling = std::make_shared<FirstOrderRetractionSampling<dim>>(
                    m_env, m_graph, m_sampler, std::dynamic_pointer_cast<Constraint<dim>>(m_validityChecker), m_validityChecker,
                    m_samplingAttempts, m_samplingDist, m_taskFrame);
            } else {
                Logging::error("This module nesting ist not possible, set StraightSampling", this);
                m_sampling = std::make_shared<StraightSampling<dim>>(m_env, m_sampler, m_validityChecker);
            }
            break;
        case SamplingType::Berenson:
            if (m_validityType == ValidityCheckerType::FclSerialAndConstraint) {
                auto checker = std::dynamic_pointer_cast<ComposeValidity<dim>>(m_validityChecker);
                m_sampling = std::make_shared<BerensonSampling<dim>>(
                    m_env, m_graph, m_sampler, std::dynamic_pointer_cast<Constraint<dim>>(checker->getChecker(1)),
                    m_validityChecker, m_metric, m_samplingAttempts, m_samplingDist);
            } else if (m_validityType == ValidityCheckerType::BerensonConstraint) {
                m_sampling = std::make_shared<BerensonSampling<dim>>(
                    m_env, m_graph, m_sampler, std::dynamic_pointer_cast<Constraint<dim>>(m_validityChecker), m_validityChecker,
                    m_metric, m_samplingAttempts, m_samplingDist);
            } else {
                Logging::error("This module nesting ist not possible, set StraightSampling", this);
                m_sampling = std::make_shared<StraightSampling<dim>>(m_env, m_sampler, m_validityChecker);
            }
            break;
        default:
            m_sampling = std::make_shared<StraightSampling<dim>>(m_env, m_sampler, m_validityChecker);
            break;
    }
}

/*!
*  \brief      Save all properties of the ModuleConfigurator to the defined json file.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] true if saving was successful
*  \date       2017-10-03
*/
template <unsigned int dim>
bool ModuleConfigurator<dim>::saveConfig(const std::string &filePath) {
    // types
    nlohmann::json json;
    json["DistanceMetricType"] = static_cast<int>(m_metricType);
    json["MetricWeight"] = vectorToString<dim>(m_metricWeight);
    json["EvaluatorType"] = static_cast<int>(m_evaluatorType);
    json["QueryEvaluatorDist"] = m_queryEvalDist;
    json["EvaluatorDuration"] = m_evaluatorDuration;
    json["GraphSortCount"] = m_graphSortCount;
    json["NeighborFinderType"] = static_cast<int>(m_neighborType);
    json["PathModifierType"] = static_cast<int>(m_pathModifierType);
    json["SamplerType"] = static_cast<int>(m_samplerType);
    json["SamplerSeed"] = m_samplerSeed;
    json["SamplerGridResolution"] = m_samplerGridResolution;
    json["SamplingType"] = static_cast<int>(m_samplingType);
    json["SamplingAttempts"] = m_samplingAttempts;
    json["SamplingDist"] = m_samplingDist;
    json["MedialAxisDirs"] = m_medialAxisDirs;
    json["TrajectoryPlannerType"] = static_cast<int>(m_trajectoryType);
    json["PosRes"] = m_posRes;
    json["OriRes"] = m_oriRes;
    json["ValidityType"] = static_cast<int>(m_validityType);

    return ui::save(filePath, json);
}

/*!
*  \brief      Loads all properties of the ModuleConfigurator from the defined json file.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] true if loading was successful
*  \date       2017-10-03
*/
template <unsigned int dim>
bool ModuleConfigurator<dim>::loadConfig(const std::string &filePath) {
    nlohmann::json json = ui::loadJson(filePath);
    if (json.empty())
        return false;

    m_metricType = static_cast<DistanceMetricType>(json["DistanceMetricType"].get<int>());
    m_metricWeight = stringToVector<dim>(json["MetricWeight"].get<std::string>());
    m_evaluatorType = static_cast<EvaluatorType>(json["EvaluatorType"].get<int>());
    m_queryEvalDist = json["QueryEvaluatorDist"].get<double>();
    m_evaluatorDuration = json["EvaluatorDuration"].get<size_t>();
    m_graphSortCount = json["GraphSortCount"].get<size_t>();
    m_neighborType = static_cast<NeighborFinderType>(json["NeighborFinderType"].get<int>());
    m_pathModifierType = static_cast<PathModifierType>(json["PathModifierType"].get<int>());
    m_samplerType = static_cast<SamplerType>(json["SamplerType"].get<int>());
    m_samplerSeed = json["SamplerSeed"].get<std::string>();
    m_samplerGridResolution = json["SamplerGridResolution"].get<double>();
    m_samplingType = static_cast<SamplingType>(json["SamplingType"].get<int>());
    m_samplingAttempts = json["SamplingAttempts"].get<size_t>();
    m_samplingDist = json["SamplingDist"].get<double>();
    m_medialAxisDirs = json["MedialAxisDirs"].get<size_t>();
    m_trajectoryType = static_cast<TrajectoryPlannerType>(json["TrajectoryPlannerType"].get<int>());
    m_posRes = json["PosRes"].get<double>();
    m_posRes = json["OriRes"].get<double>();
    m_validityType = static_cast<ValidityCheckerType>(json["ValidityType"].get<int>());
    initializeModules();

    return true;
}

template <unsigned int dim>
void ModuleConfigurator<dim>::resetModules() {
    m_parameterModified = true;
}

/*!
*  \brief      Sets the Environment
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setEnvironment(const std::shared_ptr<Environment> &environment) {
    m_env = environment;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the constraint properties
*  \author     Sascha Kaden
*  \param[in]  euclidean constraint vector
*  \date       2018-01-08
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setConstraintProperties(const std::pair<Vector6, Vector6> &C, const Transform &taskFrame,
                                                      const double epsilon) {
    m_taskFrameC = C;
    m_constraintEpsilon = epsilon;
    m_taskFrame = taskFrame;
}

/*!
*  \brief      Sets the DistanceMetricType
*  \author     Sascha Kaden
*  \param[in]  DistanceMetricType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setMetricType(DistanceMetricType type) {
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
void ModuleConfigurator<dim>::setEvaluatorType(EvaluatorType type) {
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
void ModuleConfigurator<dim>::setEvaluatorProperties(double queryEvaluatorDist, size_t duration,
                                                     const std::pair<Vector6, Vector6> &C) {
    m_queryEvalDist = queryEvaluatorDist;
    m_evaluatorDuration = duration;
    m_goalC = C;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the Graph sort count
*  \author     Sascha Kaden
*  \param[in]  count
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setGraphSortCount(size_t count) {
    m_graphSortCount = count;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the NeighborFinderType
*  \author     Sascha Kaden
*  \param[in]  NeighborFinderType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setNeighborFinderType(NeighborFinderType type) {
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
void ModuleConfigurator<dim>::setPathModifierType(PathModifierType type) {
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
void ModuleConfigurator<dim>::setSamplerType(SamplerType type) {
    m_samplerType = type;
    m_parameterModified = true;
}

template <unsigned int dim>
void ModuleConfigurator<dim>::setSamplerProperties(const std::string &seed, double gridResolution) {
    m_samplerSeed = seed;
    if (gridResolution > 0)
        m_samplerGridResolution = gridResolution;
    else
        Logging::warning("SamplerGridResolution has to be > 0", this);
}

/*!
*  \brief      Sets the SamplingType
*  \author     Sascha Kaden
*  \param[in]  SamplingType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setSamplingType(SamplingType type) {
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
void ModuleConfigurator<dim>::setSamplingProperties(size_t samplingAttempts, double samplingDist, size_t medialAxisDirs) {
    m_samplingAttempts = samplingAttempts;
    m_samplingDist = samplingDist;
    m_medialAxisDirs = medialAxisDirs;
    m_parameterModified = true;
}

/*!
*  \brief      Sets the TrajectoryPlannerType
*  \author     Sascha Kaden
*  \param[in]  TrajectoryPlannerType
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setTrajectoryType(TrajectoryPlannerType type) {
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
void ModuleConfigurator<dim>::setTrajectoryProperties(double posRes, double oriRes) {
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
*  \brief      Sets the ValidityCheckerType
*  \author     Sascha Kaden
*  \param[in]  ValidityCheckerTypeC
*  \date       2017-10-03
*/
template <unsigned int dim>
void ModuleConfigurator<dim>::setValidityCheckerType(ValidityCheckerType type) {
    m_validityType = type;
    m_parameterModified = true;
}

template <unsigned int dim>
void ModuleConfigurator<dim>::setGoalPose(const Vector6 goalPose) {
    m_goalPose = goalPose;
}

/*!
*  \brief      Return the pointer to the Environment instance.
*  \author     Sascha Kaden
*  \param[out] Evaluator
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Environment> ModuleConfigurator<dim>::getEnvironment() {
    return m_env;
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
*  \brief      Return the pointer to the Graph B instance.
*  \author     Sascha Kaden
*  \param[out] Graph
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> ModuleConfigurator<dim>::getGraphB() {
    initializeModules();
    return m_graphB;
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
*  \brief      Return the pointer to the ValidityChecker instance.
*  \author     Sascha Kaden
*  \param[out] ValidityChecker
*  \date       2017-05-22
*/
template <unsigned int dim>
std::shared_ptr<ValidityChecker<dim>> ModuleConfigurator<dim>::getValidityChecker() {
    initializeModules();
    return m_validityChecker;
}

/*!
*  \brief      Generate PlannerOptions and return them.
*  \author     Sascha Kaden
*  \param[out] PlannerOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
MPOptions<dim> ModuleConfigurator<dim>::getPlannerOptions() {
    initializeModules();
    return MPOptions<dim>(m_validityChecker, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate PRMOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  range size
*  \param[out] PRMOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
PRMOptions<dim> ModuleConfigurator<dim>::getPRMOptions(double rangeSize) {
    initializeModules();
    return PRMOptions<dim>(rangeSize, m_validityChecker, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate RRTOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  step size
*  \param[out] RRTOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
RRTOptions<dim> ModuleConfigurator<dim>::getRRTOptions(double stepSize) {
    initializeModules();
    return RRTOptions<dim>(stepSize, m_validityChecker, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

/*!
*  \brief      Generate SRTOptions and return them.
*  \author     Sascha Kaden
*  \param[in]  number of trees
*  \param[out] SRTOptions
*  \date       2017-05-22
*/
template <unsigned int dim>
SRTOptions<dim> ModuleConfigurator<dim>::getSRTOptions(unsigned int nbOfTrees) {
    initializeModules();
    return SRTOptions<dim>(nbOfTrees, m_validityChecker, m_metric, m_evaluator, m_pathModifier, m_sampling, m_trajectory);
}

} /* namespace ippp */

#endif    // MODULECONFIGURATOR_HPP
