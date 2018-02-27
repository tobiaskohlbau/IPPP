#include <chrono>
#include <iomanip>
#include <thread>

#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

double maxLength = 500;
size_t imageCount = 0;
const unsigned int dim = 7;

std::shared_ptr<Environment> generateEnvironment() {
    EnvironmentConfigurator envConfigurator;

    envConfigurator.setWorkspaceProperties(AABB(Vector3(-10000, -10000, -10000), Vector3(10000, 10000, 10000)));
    // envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/obstacle400x400x800.obj", util::Vecd(-420, -400, 100, 0, 0,
    // util::toRad(90)));
    // envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/obstacle400x400x800.obj", util::Vecd(420, -400, 100, 0, 0,
    // util::toRad(90)));

    Vector7 minRobotBound = util::Vecd(-170, -120, -170, -120, -170, -120, -175);
    Vector7 maxRobotBound = util::Vecd(170, 120, 170, 120, 170, 120, 175);
    minRobotBound = util::toRad<7>(minRobotBound);
    maxRobotBound = util::toRad<7>(maxRobotBound);
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::vector<std::string> jointModelFiles = {
        FLAGS_assetsDir + "/robotModels/iiwa/link1.obj", FLAGS_assetsDir + "/robotModels/iiwa/link2.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link3.obj", FLAGS_assetsDir + "/robotModels/iiwa/link4.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link5.obj", FLAGS_assetsDir + "/robotModels/iiwa/link6.obj",
        FLAGS_assetsDir + "/robotModels/iiwa/link7.obj"};
    envConfigurator.setRobotBaseModelFile(FLAGS_assetsDir + "/robotModels/iiwa/link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);

    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes = {DofType::joint, DofType::joint, DofType::joint, DofType::joint,
                                     DofType::joint, DofType::joint, DofType::joint};
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, maxRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    auto linkTransforms = util::convertPosesToTransforms(linkOffsets);
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles, linkTransforms);

    // envConfigurator.saveConfig("KukaEnvConfig.json");
    return envConfigurator.getEnvironment();
}

std::shared_ptr<Planner<dim>> generatePlanner(std::shared_ptr<Environment> env, const std::pair<Vector6, Vector6>& C,
                                              const Transform& taskFrame) {
    // properties
    double stepSize = 0.5 + (dim / 5);
    size_t graphSortCount = 2500;
    size_t attempts = 50;

    // standard modules
    auto alwaysValid = std::make_shared<AlwaysTrueValidity<dim>>(env);
    auto trajectory = std::make_shared<LinearTrajectory<dim>>(env, 1, 0.01);
    auto metric = std::make_shared<L2Metric<dim>>();
    auto neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(metric);
    auto graph = std::make_shared<Graph<dim>>(graphSortCount, neighborFinder);
    auto sampler = std::make_shared<SamplerUniform<dim>>(env, "sadfsdafasdf4332154sdaf");

    // constraint
    auto stilmanConstraint = std::make_shared<StilmanConstraint<dim>>(env, taskFrame, C, IPPP_EPSILON);
    auto berensonConstraint = std::make_shared<BerensonConstraint<dim>>(env, taskFrame, C);

    // sampler
    auto TS =
        std::make_shared<TangentSpaceSampling<dim>>(env, stilmanConstraint, sampler, attempts, graph, stepSize, C, taskFrame);
    auto FOR = std::make_shared<FirstOrderRetractionSampling<dim>>(env, stilmanConstraint, sampler, attempts, graph, taskFrame);
    auto BS = std::make_shared<BerensonSampling<dim>>(env, berensonConstraint, sampler, attempts);

    auto nodeCut = std::make_shared<NodeCutPathModifier<dim>>(env, trajectory, stilmanConstraint);
    auto dummyModifier = std::make_shared<DummyPathModifier<dim>>();
    // evaluator
    std::vector<std::shared_ptr<Evaluator<dim>>> evaluators;
    evaluators.push_back(std::make_shared<TreeQueryEvaluator<dim>>(metric, graph, trajectory, stilmanConstraint, stepSize));
    evaluators.push_back(std::make_shared<TimeEvaluator<dim>>(50));
    auto evaluator = std::make_shared<ComposeEvaluator<dim>>(evaluators, ComposeType::OR);

    RRTOptions<dim> options(stepSize, stilmanConstraint, metric, evaluator, nodeCut, BS, trajectory);

    return std::make_shared<RRTStar<dim>>(env, options, graph);
}

bool run(std::shared_ptr<Environment> env, std::shared_ptr<Planner<dim>>& planner, const Vector<dim>& start,
         const Vector<dim>& goal) {
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(env->getRobot());

    auto startTime = std::chrono::system_clock::now();
    bool connected = planner->computePath(start, goal, 8000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    if (connected) {
        Logging::info("Init and goal could be connected! \n", "Example");
        auto path = planner->getPath(0.001, 0.001);

        auto json = jsonSerializer::serialize<dim>(path);
        ui::save("kukaPath.json", json);
    }
    return connected;
}

bool test2DSerialRobot() {
    const double eps = 2.5;
    // environment configuration
    auto environment = generateEnvironment();
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());

    // display configuration
    // auto test = util::toRad<dim>(util::Vecd(0, 0, 0, 0, 0, 0, 0));
    // Vector<dim> test = util::toRad<dim>(util::Vecd(-90, 110, -30, 30, 30, -85, 0));
    // Transform tcp = serialRobot->getTcp(serialRobot->getJointTrafos(test));
    // std::cout << "TCP:" << std::endl << util::transformToVec(tcp) << std::endl;
    // serialRobot->saveMeshConfig(test);
    // tcp = serialRobot->getTcp(serialRobot->getJointTrafos(goal));
    // std::cout << "test2:" << std::endl << tcp.matrix() << std::endl;

    // std::cout << "test:" << constraint->calc(test) << std::endl;
    // std::cout << "test2:" << constraint->calc(test2) << std::endl;

    Vector<dim> start = util::toRad<dim>(util::Vecd(0, 0, 0, 0, 0, 0, 0));
    Vector<dim> goal = util::toRad<dim>(util::Vecd(-90, 110, -30, 30, 30, -85, 0));
    Vector6 Cmin, Cmax;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;
    std::shared_ptr<Planner<dim>> planner;
    bool connected = false;

    // case 1: fixed x
    Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -0.1, -0.1, -IPPP_MAX);
    Cmax = util::Vecd(IPPP_MAX, IPPP_MAX, IPPP_MAX, 0.1, 0.1, IPPP_MAX);
    C = std::make_pair(Cmin, Cmax);
    taskFrame = util::poseVecToTransform(util::Vecd(0, 0, 0, 0, 0, 0));

    planner = generatePlanner(environment, C, taskFrame);
    connected = run(environment, planner, start, goal);

    return connected;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Logging::setLogLevel(LogLevel::debug);

    test2DSerialRobot();
    std::string str;
    std::cin >> str;
}
