#include <chrono>
#include <iomanip>
#include <thread>

#include <ConfigurationMA.h>
#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

std::vector<std::string> m_seeds;

size_t imageCount = 0;
const double maxLength = 500;

void initialize() {
    m_seeds = ConfigurationMA::getSeeds();
}


template <unsigned int dim>
std::shared_ptr<Environment> generateEnvironment() {
    AABB workspceBounding(Vector3(-500, -500, -1), Vector3(500, 500, 1));
    Vector<dim> min = Vector<dim>::Constant(dim, 1, -util::pi());
    Vector<dim> max = -min;
    ModelFactoryFcl factory;

    DhParameter dhParameter(0, maxLength / dim);
    std::vector<Joint> joints;
    for (size_t i = 0; i < dim; ++i) {
        auto model = factory.createModelFromFile(FLAGS_assetsDir + "/robotModels/2D/2dLineDim" + std::to_string(dim) + ".obj");
        joints.push_back(Joint(min[i], max[i], dhParameter, model));
    }
    std::vector<DofType> dofs(dim, DofType::joint);
    auto robot = std::make_shared<SerialRobot>(dim, joints, dofs);

    auto environment = std::make_shared<Environment>(workspceBounding, robot);
    return environment;
}

template <unsigned int dim>
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
    auto sampler = std::make_shared<SamplerUniformBiased<dim>>(env, graph, "sadfsdafasdf4332154sdaf");

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
    evaluators.push_back(std::make_shared<TreeConfigEvaluator<dim>>(metric, graph, trajectory, stilmanConstraint, stepSize));
    evaluators.push_back(std::make_shared<TimeEvaluator<dim>>(50));
    auto evaluator = std::make_shared<ComposeEvaluator<dim>>(evaluators, ComposeType::OR);

    RRTOptions<dim> options(stepSize, stilmanConstraint, metric, evaluator, nodeCut, BS, trajectory);

    return std::make_shared<RRTStar<dim>>(env, options, graph);
}

template <unsigned int dim>
void displayConfig(const Vector<dim> config, SerialRobot& serialRobot, cv::Mat image, const Vector2i& offset) {
    drawing::drawSerialRobot2D<dim>(config, serialRobot, image, offset, Vector3i(0, 0, 255));
    cv::imshow("pathPlanner", image);

    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << imageCount++;
    cv::imwrite("images/" + ss.str() + ".jpg", image);
    cv::waitKey(2);
}

template <unsigned int dim>
bool run(std::shared_ptr<Environment> env, std::shared_ptr<Planner<dim>>& planner, const Vector<dim>& start,
    const Vector<dim>& goal) {
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(env->getRobot());
    auto workspace2D = cad::create2dspace(env->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    auto startTime = std::chrono::system_clock::now();
    bool connected = planner->computePath(start, goal, 8000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    if (connected) {
        Logging::info("Init and goal could be connected! \n", "Example");
        for (const auto& config : planner->getPath(10, 0.03))
            displayConfig<dim>(config, *serialRobot, image.clone(), workspace2D.second);
    } /*else {
      auto graph = planner->getGraph();
      for (const auto& node : graph->getNodes())
      displayConfig<dim>(node->getValues(), *serialRobot, image.clone(), workspace2D.second);
      }*/
    return connected;
}

template <unsigned int dim>
bool test2DSerialRobot() {
    std::vector<double> testAngles = { 35, 55, 75 };

    const double eps = 2.5;
    // environment configuration
    auto environment = generateEnvironment<dim>();
    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());

    // display configuration
    // auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    // cv::Mat image = drawing::eigenToCV(workspace2D.first);
    // cv::cvtColor(image, image, CV_GRAY2BGR);
    // auto test = util::toRad<dim>(Vector4(125, 0, 0, 0));
    // Transform tcp = serialRobot->getTcp(serialRobot->getJointTrafos(test));
    // std::cout << "test:" << std::endl << tcp.matrix() << std::endl;
    // tcp = serialRobot->getTcp(serialRobot->getJointTrafos(goal));
    // std::cout << "test2:" << std::endl << tcp.matrix() << std::endl;
    // auto constraint = std::make_shared<StilmanConstraint<dim>>(environment, taskFrame2, vecC2, 10);
    // std::cout << "test:" << constraint->calc(test) << std::endl;
    // std::cout << "test2:" << constraint->calc(test2) << std::endl;

    Vector<dim> start = Vector<dim>::Zero();
    Vector<dim> goal = Vector<dim>::Zero();
    Vector6 Cmin, Cmax;
    std::pair<Vector6, Vector6> C;
    Transform taskFrame;
    std::shared_ptr<Planner<dim>> planner;
    bool connected = false;
    double q1Angle;

    // case 1: fixed x
    q1Angle = util::toRad(35);
    Cmin = util::Vecd(-eps, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
    Cmax = util::Vecd(eps, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX);
    C = std::make_pair(Cmin, Cmax);
    taskFrame = util::toTransform(util::Vecd(maxLength * std::cos(q1Angle), 0, 0, 0, 0, 0));

    start[0] = -q1Angle;
    goal[0] = q1Angle;
    planner = generatePlanner<dim>(environment, C, taskFrame);
    connected = run<dim>(environment, planner, start, goal);

    // case 2: fixed y
    q1Angle = util::toRad(35);
    Cmin = util::Vecd(-IPPP_MAX, -eps, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
    Cmax = util::Vecd(IPPP_MAX, eps, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX);
    C = std::make_pair(Cmin, Cmax);
    taskFrame = util::toTransform(util::Vecd(0, maxLength * std::cos(q1Angle), 0, 0, 0, 0));

    start[0] = q1Angle + util::toRad(90);
    goal[0] = -q1Angle + util::toRad(90);
    planner = generatePlanner<dim>(environment, C, taskFrame);
    connected = run<dim>(environment, planner, start, goal);

    // case 3: fixed orientation z
    q1Angle = util::toRad(90);
    Cmin = util::Vecd(-IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -0.1);
    Cmax = util::Vecd(IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, IPPP_MAX, 0.1);
    C = std::make_pair(Cmin, Cmax);
    taskFrame = util::toTransform(util::Vecd(0, 0, 0, 0, 0, 0));

    start[0] = -q1Angle;
    goal[0] = q1Angle;
    start[dim - 1] = q1Angle;
    goal[dim - 1] = -q1Angle;
    planner = generatePlanner<dim>(environment, C, taskFrame);
    connected = run<dim>(environment, planner, start, goal);

    return connected;
}

void mobileRobot() {

}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Logging::setLogLevel(LogLevel::debug);

    test2DSerialRobot<4>();
    test2DSerialRobot<5>();
    test2DSerialRobot<6>();
    test2DSerialRobot<7>();
    test2DSerialRobot<8>();
    // test2DSerialRobot<9>();
    std::string str;
    std::cin >> str;
}
