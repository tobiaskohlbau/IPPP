#include <chrono>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <gflags/gflags.h>
#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>
#include <Drawing2D.hpp>

using namespace ippp;

DEFINE_string(assetsDir, "../assets",
	"assets directory");

Mesh generateMap() {
    const unsigned int dim = 2;
    Vector2 min(0, 0);
    Vector2 max(1000, 1000);
    std::shared_ptr<Sampler<dim>> sampler(new SamplerRandom<dim>(min, max));

    MapGenerator<dim> mapGenerator(min, max, sampler);
    auto meshes = mapGenerator.generateMap(80, Vector2(80, 80), Vector2(10, 10));
    auto mesh = cad::mergeMeshes(meshes);
    cad::exportCad(cad::ExportFormat::OBJ, "obstacle", mesh);
    return mesh;
}

bool testTriangleRobot() {
    const unsigned int dim = 3;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(dim, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    envConfigurator.setRobotType(RobotType::Triangle2D, FLAGS_assetsDir + "/robotModels/simpleTriangleRobot.obj");
    envConfigurator.addObstaclePath("obstacle.obj");
    envConfigurator.saveConfig("envConfigTriangle.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setCollisionType(CollisionType::Dim2Triangle);
    creator.setEvaluatorType(EvaluatorType::QueryOrTime);
    creator.setEvaluatorProperties(50, 60);
    creator.setSamplingType(SamplingType::Straight);
    creator.saveConfig("moduleConfigTriangle.json");

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

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();
    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector3> path = planner->getPath(80, 5);

        drawing::drawTrianglePath(path, environment->getRobot()->getBaseModel()->m_mesh, image, Eigen::Vector3i(0, 0, 255), 2);
        //JsonSerializer writer;
        //writer.write<dim>(path, "test.txt");

        cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
        cv::imshow("pathPlanner", image);
        cv::imwrite("result.png", image);
        cv::waitKey(0);
        return true;
    }
    return false;
}

bool test2DSerialRobot() {
    const unsigned int dim = 5;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(2, AABB(Vector3(0, 0, -1), Vector3(1000, 1000, 1000)));
    envConfigurator.setRobotType(RobotType::Serial2D);
    envConfigurator.setFactoryType(FactoryType::ModelPQP);
    envConfigurator.saveConfig("envConfigSerial2D.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();
    environment->getRobot()->setPose(util::Vecd(200, 500, 0, 0, 0, 0));

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setGraphSortCount(3000);
    creator.setCollisionType(CollisionType::PQP);
    creator.setTrajectoryProperties(10, 0.01);
    creator.setEvaluatorType(EvaluatorType::Query);
    creator.setEvaluatorProperties(2, 60);
    creator.setSamplerType(SamplerType::SamplerUniform);
    creator.setSamplingType(SamplingType::Straight);
    creator.saveConfig("moduleConfigTriangle.json");

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::make_shared<PRM<dim>>(environment, creator.getPRMOptions(30), creator.getGraph());
    planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(5), creator.getGraph());
    // planner = std::make_shared<RRT<dim>>(environment, creator.getRRTOptions(50), creator.getGraph());
    // planner = std::make_shared<SRT<dim>>(environment, creator.getSRTOptions(20), creator.getGraph());

    auto startTime = std::chrono::system_clock::now();
    Vector5 start =
        util::Vecd(-55 * util::toRad(), -55 * util::toRad(), -55 * util::toRad(), -55 * util::toRad(), -55 * util::toRad());
    Vector5 goal = util::Vecd(55 * util::toRad(), 55 * util::toRad(), 55 * util::toRad(), 55 * util::toRad(), 55 * util::toRad());
    bool connected = planner->computePath(start, goal, 500, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::resizeWindow("pathPlanner", 600, 600);

    if (connected) {
        Logging::info("Init and goal could be connected!", "Example");
        std::vector<Vector<dim>> path = planner->getPath(10, 0.1);

        for (auto& config : path) {
            cv::Mat imageCopy = image.clone();
            drawing::drawSerialRobot2D<dim>(config, std::dynamic_pointer_cast<SerialRobot2D>(environment->getRobot()), imageCopy,
                                            Eigen::Vector3i(0, 0, 255), 2);
            cv::imshow("pathPlanner", imageCopy);
            cv::waitKey(0);
        }
    } else {
        drawing::drawSerialRobot2D<dim>(goal, std::dynamic_pointer_cast<SerialRobot2D>(environment->getRobot()), image,
                                        Eigen::Vector3i(0, 0, 255), 2);
    }

    cv::imshow("pathPlanner", image);
    cv::waitKey(0);

    return connected;
}

void testPointRobot() {
    const unsigned int dim = 2;

    EnvironmentConfigurator envConfigurator;
    envConfigurator.setWorkspaceProperties(dim, AABB(Vector3(0, 0, 0), Vector3(1000, 1000, 1000)));
    //envConfigurator.addObstaclePath("../../assets/spaces/random2D.obj");
    envConfigurator.setRobotType(RobotType::Point);
    envConfigurator.saveConfig("envConfig.json");
    std::shared_ptr<Environment> environment = envConfigurator.getEnvironment();

    ModuleConfigurator<dim> creator;
    creator.setEnvironment(environment);
    creator.setCollisionType(CollisionType::Dim2);
    creator.setGraphSortCount(3000);
    creator.setEvaluatorType(EvaluatorType::Query);
    creator.setEvaluatorProperties(50, 10);
    creator.setSamplerType(SamplerType::SamplerRandom);
    creator.setSamplerProperties("slkasjdfsaldfj234;lkj", 1);
    creator.setSamplingType(SamplingType::NearObstacle);
    creator.setSamplingProperties(10, 80);
    creator.saveConfig("moduleConfig.json");
    creator.loadConfig("moduleConfig.json");

    std::shared_ptr<ippp::Planner<dim>> planner;
    // planner = std::make_shared<EST<dim>>(environment, creator.getPlannerOptions(), creator.getGraph());
    // planner = std::make_shared<PRM<dim>>(environment, creator.getPRMOptions(40), creator.getGraph());
    planner = std::make_shared<RRTStar<dim>>(environment, creator.getRRTOptions(35), creator.getGraph());
    // planner = std::make_shared<RRT<dim>>(environment, creator.getRRTOptions(50), creator.getGraph());
    // planner = std::make_shared<SRT<dim>>(environment, creator.getSRTOptions(20), creator.getGraph());

    // compute the tree
    auto startTime = std::chrono::system_clock::now();
    Vector2 start(10.0, 10.0);
    Vector2 goal(990.0, 990.0);
    bool connected = planner->computePath(start, goal, 1000, 3);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
    std::cout << "Computation time: " << std::chrono::milliseconds(duration).count() / 1000.0 << std::endl;
    std::vector<std::shared_ptr<Node<dim>>> nodes = planner->getGraphNodes();

    Eigen::MatrixXi workspace2D = cad::create2dspace(environment->getSpaceBoundary(), 255);
    std::vector<Mesh> meshes;
    for (const auto& obstacle : environment->getObstacles())
        meshes.push_back(obstacle->m_mesh);
    cad::drawTriangles(workspace2D, meshes, 50);
    cv::Mat image = drawing::eigenToCV(workspace2D);
    cv::cvtColor(image, image, CV_GRAY2BGR);

    drawing::drawGraph2D(nodes, image, Eigen::Vector3i(125, 125, 200), Eigen::Vector3i(125, 125, 200), 1);
    drawing::drawTree2D(nodes, image, Eigen::Vector3i(0, 0, 255), Eigen::Vector3i(125, 125, 200), 1);

    if (connected) {
        std::vector<Vector2> pathPoints = planner->getPath();
        
        JsonSerializer serializer;
        std::string string = serializer.serialize<dim>(pathPoints);
        ui::save("test.json", string);
        drawing::drawPath2D(pathPoints, image, Eigen::Vector3i(255, 0, 0), 2);
    }

    cv::namedWindow("pathPlanner", CV_WINDOW_AUTOSIZE);
    cv::imshow("pathPlanner", image);
    cv::imwrite("result.png", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
	gflags::ParseCommandLineFlags(&argc, &argv, true);

    Logging::setLogLevel(LogLevel::trace);

    // testTriangleRobot();
    // test2DSerialRobot();
    testPointRobot();
}
