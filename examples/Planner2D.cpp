#include <chrono>

#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

void generateMap(AABB workspace) {
    Vector2 min = Vector2(workspace.min()[0], workspace.min()[1]);
    Vector2 max = Vector2(workspace.max()[0], workspace.max()[1]);
    auto sampler = std::make_shared<SamplerRandom<2>>(std::make_pair(min, max));

    cad::MapGenerator<2> mapGenerator(workspace, sampler);
    auto meshes = mapGenerator.generateMap(80, Vector2(10, 10), Vector2(80, 80));
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", cad::mergeMeshes(meshes));
}

bool testTriangleRobot() {
    const unsigned int dim = 3;

    EnvironmentConfigurator envConfigurator;
    AABB workspaceBound(Vector3(0, 0, 0), Vector3(1000, 1000, 1));
    envConfigurator.setWorkspaceProperties(workspaceBound);
    envConfigurator.setRobotType(RobotType::Mobile2D);
    envConfigurator.setRobotBaseModelFile(FLAGS_assetsDir + "/robotModels/simpleTriangleRobot.obj");
    //    generateMap(workspaceBound);
    //  envConfigurator.addObstacle("obstacle.obj");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setValidityCheckerType(ValidityCheckerType::FclMobile);
    creator.setEvaluatorType(EvaluatorType::TreeConnect);
    creator.setEvaluatorProperties(40, 60);

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::make_shared<PRM<dim>>(environment, creator.getPRMOptions(30), creator.getGraph());
    // planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(40), creator.getGraph());
    planner =
        std::make_shared<RRTStarConnect<dim>>(environment, creator.getRRTOptions(40), creator.getGraph(), creator.getGraphB());
    // planner = std::make_shared<RRT<dim>>(environment, creator.getRRTOptions(50), creator.getGraph());
    // planner = std::make_shared<SRT<dim>>(environment, creator.getSRTOptions(20), creator.getGraph());

    Vector3 start(50, 50, 0);
    Vector3 goal(900, 900, 50 * util::toRad());
    bool connected = planner->computePath(start, goal, 5000, 3);

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : environment->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        auto path = planner->getPath(80, util::toRad(90));
        drawing::drawTrianglePath(image, path, environment->getRobot()->getBaseModel()->m_mesh, workspace2D.second,
                                  Vector3i(0, 0, 255));

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        return true;
    }
    return false;
}

bool test2DSerialRobot() {
    const unsigned int dim = 3;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(AABB(Vector3(0, 0, -1), Vector3(1000, 1000, 1000)));
    envConfigurator.setRobotType(RobotType::Serial);
    envConfigurator.setFactoryType(FactoryType::ModelFCL);
    Vector3 min(-util::pi(), -util::pi(), -util::pi());
    Vector3 max(util::pi(), util::pi(), util::pi());
    envConfigurator.setRobotBaseProperties(dim, std::vector<DofType>(dim, DofType::jointRot), std::make_pair(min, max));
    std::vector<DhParameter> dhParameters(3, DhParameter(0, 100));
    std::vector<std::string> jointModelFiles(3, std::string(FLAGS_assetsDir + "/robotModels/2dLine.obj"));
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();
    environment->getRobot()->setPose(util::Vecd(200, 500, 0, 0, 0, 0));

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setValidityCheckerType(ValidityCheckerType::FclSerial);
    creator.setTrajectoryProperties(10, 0.01);
    creator.setEvaluatorType(EvaluatorType::TreeConfigOrTime);
    creator.setEvaluatorProperties(1, 60);
    creator.setSamplerType(SamplerType::Uniform);
    creator.setSamplingType(SamplingType::Straight);

    auto planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(5), creator.getGraph());

    Vector3 start(-55 * util::toRad(), -55 * util::toRad(), -55 * util::toRad());
    Vector3 goal(55 * util::toRad(), 55 * util::toRad(), 55 * util::toRad());
    bool connected = planner->computePath(start, goal, 100, 3);

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : environment->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));
    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);

    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector<dim>> path = planner->getPath(10, 0.1);
        auto data = jsonSerializer::serialize<dim>(path);
        ui::save("2dSerialRobotPath.json", data);

        for (const auto& config : path) {
            cv::Mat imageCopy = image.clone();
            drawing::drawSerialRobot2D<dim>(config, *serialRobot, imageCopy, workspace2D.second, Vector3i(0, 0, 255));
            cv::imshow("pathPlanner", imageCopy);
        }
    } else {
        drawing::drawSerialRobot2D<dim>(goal, *serialRobot, image, workspace2D.second, Vector3i(0, 0, 255));
    }

    cv::imshow("pathPlanner", image);

    return connected;
}

void testPointRobot() {
    const unsigned int dim = 2;
    Vector6 min = util::Vecd(-30, -30, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX, -IPPP_MAX);
    auto C = std::make_pair(min, -min);

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/random2D.obj");
    envConfigurator.setRobotType(RobotType::Point2D);
    auto env = envConfigurator.getEnvironment();

    double stepSize = 40;
    ModuleConfigurator<dim> creator;
    creator.setEnvironment(env);
    creator.setPathModifierType(PathModifierType::NodeCut);
    creator.setValidityCheckerType(ValidityCheckerType::Dim2);
    creator.setGraphSortCount(3000);
    creator.setEvaluatorType(EvaluatorType::TreeConnect);
    creator.setEvaluatorProperties(stepSize, 30);
    creator.setSamplerType(SamplerType::UniformBiased);
    creator.setSamplerProperties("slkasjdfsaldfj234;lkj", 1);
    creator.setSamplingProperties(10, 80);

    auto planner =
        std::make_shared<RRTStarConnect<dim>>(env, creator.getRRTOptions(stepSize), creator.getGraph(), creator.getGraphB());
    Vector2 start(150, 200);
    //Vector2 goal(730, 350);
    Vector2 goal(950, 950);

    bool connected = planner->computePath(start, goal, 500, 3);

    auto workspace2D = cad::create2dspace(env->getSpaceBoundary(), 255);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    for (const auto& obstacle : env->getObstacles())
        drawing::drawPolygons(image, obstacle->model->m_mesh, obstacle->getPose(), workspace2D.second, Vector3i(50, 50, 50));

    if (connected) {
        drawing::drawPath2D(image, planner->getPath(), Vector3i(255, 0, 0), 2);
        planner->optimize(5000, 10);
        drawing::drawPath2D(image, planner->getPath(), Vector3i(0, 0, 255), 2);
        std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
        drawing::drawTree2D(image, nodes, Vector3i(0, 0, 255), Vector3i(125, 125, 200), 1);
    }

    cv::namedWindow("Point2D", CV_WINDOW_AUTOSIZE);
    cv::imshow("Point2D", image);
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::debug);

    // while (!testTriangleRobot());
    // testTriangleRobot();
    // test2DSerialRobot();
    testPointRobot();

    ui::save("Stats.json", Stats::serialize());
    Stats::writeData(std::cout);
    cv::waitKey(0);
}
