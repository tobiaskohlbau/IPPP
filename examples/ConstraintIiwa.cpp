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
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/cupboard.obj", util::Vecd(-300, 0, 300, 0, 0, util::toRad(90)));
    //envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/obstacle400x400x800.obj",
    //                            util::Vecd(420, -400, 100, 0, 0, util::toRad(90)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/3D/table.obj");

    Vector7 minRobotBound = util::Vecd(-170, -120, -170, -120, -170, -120, -175);
    Vector7 maxRobotBound = util::Vecd(170, 120, 170, 120, 170, 120, 175);
    minRobotBound = util::toRad<7>(minRobotBound);
    maxRobotBound = util::toRad<7>(maxRobotBound);
    std::vector<DhParameter> dhParameters({DhParameter(util::toRad(-90), 0, 340), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(util::toRad(90), 0, 400), DhParameter(util::toRad(-90), 0, 0),
                                           DhParameter(util::toRad(-90), 0, 400), DhParameter(util::toRad(90), 0, 0),
                                           DhParameter(0, 0, 126)});
    std::string iiwa = FLAGS_assetsDir + "/robotModels/iiwa/";
    std::vector<std::string> linkModelFiles = {iiwa + "link1.obj", iiwa + "link2.obj", iiwa + "link3.obj", iiwa + "link4.obj",
                                               iiwa + "link5.obj", iiwa + "link6.obj", iiwa + "link7.obj"};
    envConfigurator.setRobotBaseModelFile(iiwa + "link0.obj");
    envConfigurator.setRobotType(RobotType::Serial);

    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    std::vector<DofType> dofTypes(7, DofType::jointRot);
    envConfigurator.setRobotBaseProperties(dim, dofTypes, std::make_pair(minRobotBound, maxRobotBound));
    std::vector<Vector6> linkOffsets(7, util::Vecd(0, 0, 0, 0, 0, 0));
    linkOffsets[0] = util::Vecd(0, 0, 150, 0, 0, 0);
    linkOffsets[1] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    linkOffsets[2] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[3] = util::Vecd(0, 0, 0, -util::halfPi(), 0, 0);
    linkOffsets[4] = util::Vecd(0, 0, 200, 0, 0, 0);
    linkOffsets[5] = util::Vecd(0, 0, 0, util::halfPi(), 0, 0);
    auto linkTransforms = util::toTransform(linkOffsets);
    envConfigurator.setSerialRobotProperties(dhParameters, linkModelFiles, linkTransforms, Transform::Identity(), Transform::Identity(),
                                             util::toTransform(util::Vecd(13, 7, 120, 0, 0, 0)),
                                             FLAGS_assetsDir + "/robotModels/wesslingHand.obj");

    envConfigurator.saveConfig("KukaEnvConfig.json");
    return envConfigurator.getEnvironment();
}

std::shared_ptr<Planner<dim>> generatePlanner(std::shared_ptr<Environment> env, const std::pair<Vector6, Vector6>& C,
                                              const Transform& taskFrame) {
    // properties
    double stepSize = util::toRad(50);
    size_t graphSortCount = 2500;
    size_t attempts = 50;

    // standard modules
    auto collision = std::make_shared<CollisionFclSerial<dim>>(env);
    auto trajectory = std::make_shared<LinearTrajectory<dim>>(env, 1, util::toRad(1));
    auto metric = std::make_shared<L2Metric<dim>>();
    auto neighborFinder = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(metric);
    auto graph = std::make_shared<Graph<dim>>(graphSortCount, neighborFinder);
    auto neighborFinderB = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(metric);
    auto graphB = std::make_shared<Graph<dim>>(graphSortCount, neighborFinderB);
    auto sampler = std::make_shared<SamplerUniformBiased<dim>>(env, graph, "sadfsdafasdf4332154sdaf");

    // constraint
    //auto stilmanConstraint = std::make_shared<StilmanConstraint<dim>>(env, taskFrame, C, IPPP_EPSILON);
    auto berensonConstraint = std::make_shared<BerensonConstraint<dim>>(env, taskFrame, C);
    std::vector<std::shared_ptr<ValidityChecker<dim>>> checkers = {collision, berensonConstraint};
    auto validityChecker = std::make_shared<ComposeValidity<dim>>(env, checkers, ComposeType::AND);

    // sampler
    //auto TS =
    //    std::make_shared<TangentSpaceSampling<dim>>(env, graph, sampler, stilmanConstraint, attempts, stepSize, C, taskFrame);
    //auto FOR = std::make_shared<FirstOrderRetractionSampling<dim>>(env, graph, sampler, stilmanConstraint, attempts, taskFrame);
    auto BS = std::make_shared<BerensonSampling<dim>>(env, sampler, berensonConstraint, attempts);

    auto nodeCut = std::make_shared<NodeCutPathModifier<dim>>(env, trajectory, berensonConstraint);
    // evaluator
    std::vector<std::shared_ptr<Evaluator<dim>>> evaluators;
    evaluators.push_back(std::make_shared<TreeConnectEvaluator<dim>>(graph, graphB, trajectory, validityChecker, util::toRad(50)));
    evaluators.push_back(std::make_shared<TimeEvaluator<dim>>(80));
    auto evaluator = std::make_shared<ComposeEvaluator<dim>>(evaluators, ComposeType::OR);

    RRTOptions<dim> options(stepSize, validityChecker, metric, evaluator, nodeCut, BS, trajectory);

    return std::make_shared<RRTStarConnect<dim>>(env, options, graph, graphB);
}

bool run(std::shared_ptr<Environment> env, std::shared_ptr<Planner<dim>>& planner, const Vector<dim>& start,
         const Vector<dim>& goal) {
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(env->getRobot());

    auto timer = std::make_shared<StatsTimeCollector>("Planning Time");
    Stats::addCollector(timer);
    timer->start();
    bool connected = planner->computePath(start, goal, 72000, 24);
    timer->stop();

    if (connected) {
        Logging::info("Init and goal could be connected! \n", "Example");
        auto path = planner->getPath(0.001, 0.001);

        //auto json = jsonSerializer::serialize<dim>(path);
        //ui::save("kukaPath.json", json);
        auto json = txtSerializer::serialize<dim>(path);
        ui::save("kukaPath.json", json);
    }
    return connected;
}

bool test2DSerialRobot() {
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

    Vector<dim> start = util::toRad<dim>(util::Vecd(90, 90, 150, 30, 35, 114, 0));
    Vector<dim> goal = util::toRad<dim>(util::Vecd(-90, 90, 150, 30, 35, 114, 0));
    Vector6 Cmin, Cmax;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;

    // case 1: fixed x
    double rad(util::toRad(5));
    Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -rad, -rad, -IPPP_MAX);
    Cmax = util::Vecd(IPPP_MAX, IPPP_MAX, IPPP_MAX, rad, rad, IPPP_MAX);
    C = std::make_pair(Cmin, Cmax);
    taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));

    auto planner = generatePlanner(environment, C, taskFrame);

    return run(environment, planner, start, goal);
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);

    test2DSerialRobot();

    Stats::writeData(std::cout);

    std::string str;
    std::cin >> str;
}
