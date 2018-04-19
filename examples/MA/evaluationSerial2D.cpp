#include <thread>

#include <ConfigurationMA.h>
#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../../assets", "assets directory");

double m_maxLength = 500;
AABB m_workspace(Vector3(-500, -500, 0), Vector3(500, 500, 1));

ConfigurationMA *m_configurationMA;
std::vector<ParamsMA> m_paramsMA;
size_t m_type;

template <unsigned int dim>
void displayConfig(const Vector<dim> config, SerialRobot &serialRobot, cv::Mat image, const Vector2i &offset) {
    drawing::drawSerialRobot2D<dim>(config, serialRobot, image, offset, Vector3i(0, 0, 255));
    cv::imshow("pathPlanner", image);
    cv::waitKey(0);
}

template <unsigned int dim>
std::shared_ptr<Environment> createEnvironment() {
    Vector<dim> min = Vector<dim>::Constant(dim, 1, -util::pi());
    Vector<dim> max = -min;
    ModelFactoryFcl factory;
    DhParameter dhParam(0, m_maxLength / dim);
    std::vector<Joint> joints;
    for (size_t i = 0; i < dim; ++i) {
        auto model = factory.createModelFromFile(FLAGS_assetsDir + "/robotModels/2D/2dLineDim" + std::to_string(dim) + ".obj");
        joints.push_back(Joint(min[i], max[i], dhParam, model));
    }
    std::vector<DofType> dofs(dim, DofType::jointRot);
    auto robot = std::make_shared<SerialRobot>(dim, joints, dofs);
    // no obstacles
    return std::make_shared<Environment>(m_workspace, robot);
}

template <unsigned int dim>
ModuleConfigurator<dim> getCreator(std::shared_ptr<Environment> env, const ParamsMA &params, const Transform &taskFrame) {
    ModuleConfigurator<dim> creator;
    creator.setEnvironment(env);
    creator.setPathModifierType(PathModifierType::NodeCut);
    creator.setValidityCheckerType(ValidityCheckerType::BerensonConstraint);
    creator.setGraphSortCount(5000);
    creator.setEvaluatorType(EvaluatorType::TreeConfigOrTime);
    creator.setEvaluatorProperties(params.stepSize, 45);
    creator.setSamplerType(params.samplerType);
    creator.setSamplingType(params.samplingType);
    creator.setSamplerProperties(params.seed, 1);
    creator.setSamplingProperties(10, params.stepSize);
    creator.setConstraintProperties(params.C, taskFrame);
    return creator;
}

template <unsigned int dim>
void planningThread(size_t startIndex, size_t endIndex, const Transform taskFrame, const Vector<dim> start,
                    const Vector<dim> goal) {
    for (auto params = m_paramsMA.begin() + startIndex; params < m_paramsMA.begin() + endIndex; ++params) {
        Stats::initializeCollectors();
        m_configurationMA->updatePropertyStats(params - m_paramsMA.begin());
        auto env = createEnvironment<dim>();
        auto creator = getCreator<dim>(env, *params, taskFrame);

        RRTStar<dim> planner(creator.getEnvironment(), creator.getRRTOptions(params->stepSize), creator.getGraph());
        planner.computePath(start, goal, 3000, 1);
        if (params->optimize)
            planner.optimize(1000, 1);

        ui::save("data.json", Stats::serialize(), 4, true);
        // ui::save("data/Dim" + std::to_string(dim) + "_Type" + std::to_string(m_type) + "_" +
        //         std::to_string(params - m_paramsMA.begin()) + ".json",
        //         Stats::serialize());
    }
}

template <unsigned int dim>
void generateConstraintProperties(size_t type, double eps, std::pair<Vector6, Vector6> &C, Transform &taskFrame,
                                  Vector<dim> &start, Vector<dim> &goal) {
    start = Vector<dim>::Zero();
    goal = Vector<dim>::Zero();
    Vector6 Cmin, Cmax;
    double q1Angle;

    if (type == 1) {    // case 1: fixed x
        q1Angle = util::toRad(35);
        Cmin = util::Vecd(-eps, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        Cmax = util::Vecd(eps, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX);
        C = std::make_pair(Cmin, Cmax);
        taskFrame = util::toTransform(util::Vecd(m_maxLength * std::cos(q1Angle), 0, 0, 0, 0, 0));
        start[0] = -q1Angle;
        goal[0] = q1Angle;
    } else if (type == 2) {    // case 2: fixed y
        q1Angle = util::toRad(35);
        Cmin = util::Vecd(-IPPP_MAX, -eps, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
        Cmax = util::Vecd(IPPP_MAX, eps, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX);
        C = std::make_pair(Cmin, Cmax);
        taskFrame = util::toTransform(util::Vecd(0, m_maxLength * std::cos(q1Angle), 0, 0, 0, 0));
        start[0] = q1Angle + util::toRad(90);
        goal[0] = -q1Angle + util::toRad(90);
    } else if (type == 3) {    // case 3: fixed orientation z
        q1Angle = util::toRad(90);
        Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -0.1);
        Cmax = util::Vecd(IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, 0.1);
        C = std::make_pair(Cmin, Cmax);
        taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));
        start[0] = -q1Angle;
        goal[0] = q1Angle;
        start[dim - 1] = q1Angle;
        goal[dim - 1] = -q1Angle;
    }
}

template <unsigned int dim>
void testSerial() {
    auto startTime = std::chrono::system_clock::now();
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;
    Vector<dim> start;
    Vector<dim> goal;
    for (size_t type = 1; type < 4; ++type) {
        m_type = type;
        generateConstraintProperties<dim>(type, 10, C, taskFrame, start, goal);
        m_configurationMA = new ConfigurationMA(RobotType::Serial, dim, true, true, C);
        std::cout << m_configurationMA->numParams() << std::endl;
        m_paramsMA = m_configurationMA->getParamsList();
        std::vector<std::thread> threads;

        size_t nbOfThreads = 1;
        size_t threadAmount = m_configurationMA->numParams() / nbOfThreads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(
                std::thread(&planningThread<dim>, i * threadAmount, (i + 1) * threadAmount, taskFrame, start, goal));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
    std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;
    std::cout << std::endl << "Evaluation time: " << duration.count() << std::endl;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::info);

    // testSerial<4>();
    // testSerial<5>();
    // testSerial<6>();
    testSerial<7>();
    // testSerial<8>();
}
