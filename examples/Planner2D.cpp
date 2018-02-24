#include <chrono>

#include <Drawing2D.hpp>
#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

DEFINE_string(assetsDir, "../assets", "assets directory");

Mesh generateMap() {
    const unsigned int dim = 2;
    Vector2 min(0, 0);
    Vector2 max(1000, 1000);
    auto sampler = std::make_shared<SamplerRandom<dim>>(std::make_pair(min, max));

    util::MapGenerator<dim> mapGenerator(std::make_pair(min, max), sampler);
    auto meshes = mapGenerator.generateMap(80, Vector2(80, 80), Vector2(10, 10));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);
    return mesh;
}

bool testTriangleRobot() {
    const unsigned int dim = 3;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.setRobotType(RobotType::Triangle2D);
    envConfigurator.setRobotBaseModelFile(FLAGS_assetsDir + "/robotModels/simpleTriangleRobot.obj");
    generateMap();
    envConfigurator.addObstacle("obstacle.obj");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setVadilityCheckerType(ValidityCheckerType::Dim2Triangle);
    creator.setEvaluatorType(EvaluatorType::QueryOrTime);
    creator.setEvaluatorProperties(50, 60);
    creator.setSamplingType(SamplingType::Straight);

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::make_shared<PRM<dim>>(environment, creator.getPRMOptions(30), creator.getGraph());
    planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(40), creator.getGraph());
    // planner = std::make_shared<RRT<dim>>(environment, creator.getRRTOptions(50), creator.getGraph());
    // planner = std::make_shared<SRT<dim>>(environment, creator.getSRTOptions(20), creator.getGraph());

    auto startTime = std::chrono::system_clock::now();
    Vector3 start(50, 50, 0);
    Vector3 goal(900, 900, 50 * util::toRad());
    bool connected = planner->computePath(start, goal, 5000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->model->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector3> path = planner->getPath(80, 5);
        drawing::drawTrianglePath(path, environment->getRobot()->getBaseModel()->m_mesh, image, workspace2D.second, Eigen::Vector3i(0, 0, 255), 2);

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
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
    envConfigurator.setRobotBaseProperties(dim, std::vector<DofType>({DofType::joint, DofType::joint, DofType::joint}),
                                           std::make_pair(min, max));
    std::vector<DhParameter> dhParameters(3, DhParameter(0, 100));
    std::vector<std::string> jointModelFiles(3, std::string(FLAGS_assetsDir + "/robotModels/2dLine.obj"));
    envConfigurator.setSerialRobotProperties(dhParameters, jointModelFiles);
    envConfigurator.saveConfig("envConfigSerial2D.json");
    envConfigurator.loadConfig("envConfigSerial2D.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();
    environment->getRobot()->setPose(util::Vecd(200, 500, 0, 0, 0, 0));

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setVadilityCheckerType(ValidityCheckerType::FclSerial);
    creator.setTrajectoryProperties(10, 0.01);
    creator.setEvaluatorType(EvaluatorType::QueryOrTime);
    creator.setEvaluatorProperties(2, 60);
    creator.setSamplerType(SamplerType::SamplerUniform);
    creator.setSamplingType(SamplingType::Straight);

    std::shared_ptr<ippp::Planner<dim>> planner;
    planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(5), creator.getGraph());

    auto startTime = std::chrono::system_clock::now();
    Vector3 start(-55 * util::toRad(), -55 * util::toRad(), -55 * util::toRad());
    Vector3 goal(55 * util::toRad(), 55 * util::toRad(), 55 * util::toRad());
    bool connected = planner->computePath(start, goal, 100, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->model->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::resizeWindow("pathPlanner", 600, 600);

    auto serialRobot = std::dynamic_pointer_cast<SerialRobot>(environment->getRobot());
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector<dim>> path = planner->getPath(10, 0.1);
        auto data = jsonSerializer::serialize<dim>(path);
        ui::save("2dSerialRobotPath.json", data);

        for (const auto& config : path) {
            cv::Mat imageCopy = image.clone();
            drawing::drawSerialRobot2D<dim>(config, *serialRobot, imageCopy, workspace2D.second, Eigen::Vector3i(0, 0, 255), 2);
            cv::imshow("pathPlanner", imageCopy);
            cv::waitKey(0);
        }
    } else {
        drawing::drawSerialRobot2D<dim>(goal, *serialRobot, image, workspace2D.second, Eigen::Vector3i(0, 0, 255), 2);
    }

    cv::imshow("pathPlanner", image);
    cv::waitKey(0);

    return connected;
}

void testPointRobot() {
    const unsigned int dim = 2;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.addObstacle(FLAGS_assetsDir + "/spaces/random2D.obj");
    envConfigurator.setRobotType(RobotType::Point);
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setVadilityCheckerType(ValidityCheckerType::Dim2);
    creator.setGraphSortCount(3000);
    creator.setEvaluatorType(EvaluatorType::TreeQuery);
    creator.setEvaluatorProperties(50, 10);
    creator.setSamplerType(SamplerType::SamplerRandom);
    creator.setSamplerProperties("slkasjdfsaldfj234;lkj", 1);
    //creator.setSamplingType(SamplingType::NearObstacle);
    creator.setSamplingProperties(10, 80);

    std::shared_ptr<ippp::Planner<dim>> planner;
    planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(35), creator.getGraph());

    auto startTime = std::chrono::system_clock::now();
    Vector2 start(10.0, 10.0);
    Vector2 goal(990.0, 990.0);
    bool connected = planner->computePath(start, goal, 1000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();

    auto workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->model->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D.first);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    drawing::drawGraph2D(nodes, image, Eigen::Vector3i(125, 125, 200), Eigen::Vector3i(125, 125, 200), 1);
    drawing::drawTree2D(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(125, 125, 200), 1);

    if (connected) {
        std::vector<Vector2> pathPoints = planner->getPath();

        auto json = jsonSerializer::serialize<dim>(pathPoints);
        ui::save("test.json", json);
        drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 2);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::trace);

    // while (!testTriangleRobot());
    // testTriangleRobot();
    //test2DSerialRobot();
    testPointRobot();
}
